"""The HUB — the PacFleet coordinator (the fleet's brain).

One node, four responsibilities:
  1. Unified picture: aggregates every robot's heartbeat into a single
     FleetStatus + a comms watchdog (stale robot -> LOST).
  2. Fusion: all robots' noisy detections -> Kalman MOT + Hungarian
     association -> global tracks with covariance (tracking.py).
  3. Decision making: a neural classifier scores each track's kinematics ->
     P(power pellet); a flagged pellet triggers an auction (auction.py) and
     the winning robot is retasked via its NavigateToWaypoint action.
  4. Operational picture: everything is rendered to RViz as markers
     (tracks, covariance ellipses, velocity vectors, assignments, labels).
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from fleet_interfaces.action import NavigateToWaypoint
from fleet_interfaces.msg import Detection, FleetStatus, RobotStatus, Track

from . import auction, threat_classifier, tracking, worldmap

FUSE_TICK = 0.1          # 10 Hz fusion
THREAT_ON = 0.70         # EMA prob to flag a power pellet...
THREAT_OFF = 0.30        # ...and to unflag it (hysteresis)
RETARGET_DIST = 2.0      # resend goal when a pursued coin moves this far
WATCHDOG_S = 3.0         # heartbeat staleness -> LOST


class Hub(Node):
    def __init__(self):
        super().__init__('hub')
        self.declare_parameter('robots', worldmap.ROBOTS)
        self.robots: list[str] = list(self.get_parameter('robots').value)

        # --- AI: load (or train on first run) the threat classifier -------
        self.classifier = threat_classifier.load_model()
        if self.classifier is None:
            self.get_logger().info('no classifier model found — training now...')
            acc = threat_classifier.train()
            self.get_logger().info(f'classifier trained (train acc {acc:.3f})')
            self.classifier = threat_classifier.load_model()

        self.tracker = tracking.Tracker()
        self.det_buffer: list[tuple[float, float]] = []
        self.status: dict[str, RobotStatus] = {}
        self.last_seen: dict[str, float] = {}
        self.lost: set[str] = set()
        self.assignments: dict[int, str] = {}      # track_id -> robot_id
        self.last_goal: dict[str, tuple[float, float]] = {}
        self.threat_flags: dict[int, bool] = {}    # track_id -> is_threat (sticky)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.nav_clients: dict[str, ActionClient] = {}
        for r in self.robots:
            self.create_subscription(
                Detection, f'/{r}/detections', self.on_detection, sensor_qos)
            self.create_subscription(
                RobotStatus, f'/{r}/status',
                lambda msg, rid=r: self.on_status(rid, msg), 10)
            self.nav_clients[r] = ActionClient(
                self, NavigateToWaypoint, f'/{r}/navigate_to_waypoint')
        self.create_subscription(Int32, '/coins/captured', self.on_captured, 10)

        self.fleet_pub = self.create_publisher(FleetStatus, 'fleet_status', 10)
        self.track_pub = self.create_publisher(Track, 'tracks', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)

        self.create_timer(FUSE_TICK, self.fuse_tick)
        self.create_timer(1.0, self.watchdog_tick)
        self.get_logger().info(
            f'HUB online, commanding fleet: {", ".join(self.robots)}')

    # ------------------------------------------------------------- ingestion
    def on_detection(self, msg: Detection):
        self.det_buffer.append((msg.position.x, msg.position.y))

    def on_status(self, robot_id: str, msg: RobotStatus):
        self.status[robot_id] = msg
        self.last_seen[robot_id] = time.monotonic()
        if robot_id in self.lost:
            self.lost.discard(robot_id)
            self.get_logger().info(f'comms restored with {robot_id}')

    def on_captured(self, msg: Int32):
        """A coin was bagged: drop its track now and free its pursuer to re-hunt
        (don't wait the ~3 s for the orphaned track to coast out)."""
        tid = int(msg.data)
        self.tracker.tracks = [t for t in self.tracker.tracks if t.id != tid]
        self.threat_flags.pop(tid, None)
        rid = self.assignments.pop(tid, None)
        if rid is not None:
            self.last_goal.pop(rid, None)
            self.get_logger().info(f'coin {tid} captured — {rid} freed to re-hunt')

    # ------------------------------------------------------------- fusion
    def fuse_tick(self):
        zs, self.det_buffer = self.det_buffer, []
        self.tracker.step(zs, FUSE_TICK)

        now = self.get_clock().now().to_msg()
        for t in self.tracker.tracks:
            self.classify(t)
            msg = Track()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            msg.track_id = t.id
            msg.status = t.status
            msg.position.x, msg.position.y = float(t.x[0]), float(t.x[1])
            msg.velocity.x, msg.velocity.y = float(t.x[2]), float(t.x[3])
            msg.covariance = [float(v) for v in t.p.flatten()]
            msg.threat_prob = t.threat_prob
            msg.label = 'pellet' if self.threat_flags.get(t.id) else 'coin'
            msg.assigned_to = self.assignments.get(t.id, '')
            self.track_pub.publish(msg)

        self.manage_tasks()
        self.publish_fleet()
        self.publish_markers()

    def classify(self, t: tracking.Track):
        feats = t.features()
        if feats is None or t.status == tracking.TENTATIVE or t.hits < 8:
            return               # only classify established tracks (~2.5 s custody);
                                 # the speed gate below is the real FP backstop
        prob = self.classifier.predict(feats)
        t.threat_prob = 0.75 * t.threat_prob + 0.25 * prob   # EMA smoothing
        flagged = self.threat_flags.get(t.id, False)
        # domain sanity gate: a power pellet is a fast, weaving mover — never flag
        # a slow drifting coin no matter what the net says (OOD protection)
        if not flagged and feats[0] < 0.8:
            return
        if not flagged and t.threat_prob > THREAT_ON:
            self.threat_flags[t.id] = True
            self.get_logger().warn(
                f'AI: track {t.id} is the POWER PELLET '
                f'(p={t.threat_prob:.2f}, speed={np.hypot(t.x[2], t.x[3]):.1f} m/s)')
        elif flagged and t.threat_prob < THREAT_OFF:
            self.threat_flags[t.id] = False
            self.get_logger().info(f'AI: track {t.id} looks like a normal coin again')

    # ------------------------------------------------------------- tasking
    def manage_tasks(self):
        """Coordinate the hunt: keep every robot chasing a coin the fleet is
        confident about, the power pellet first. Assignments are sticky — we
        only auction coins that have no pursuer, to free robots."""
        live_ids = {t.id for t in self.tracker.tracks}

        # release assignments whose coin-track died (captured or lost for good)
        for tid in [tid for tid in self.assignments if tid not in live_ids]:
            rid = self.assignments.pop(tid)
            self.last_goal.pop(rid, None)
            self.get_logger().info(f'coin {tid} gone — {rid} back to the hunt')

        # robots that went RTB or LOST drop their coin back into the pool
        for tid, rid in list(self.assignments.items()):
            st = self.status.get(rid)
            if rid in self.lost or (st and st.state == RobotStatus.RTB):
                self.assignments.pop(tid)
                self.last_goal.pop(rid, None)
                self.get_logger().warn(f'{rid} unavailable — re-auctioning coin {tid}')

        assigned = set(self.assignments.values())
        # confirmed coins are fair game; a flagged power pellet stays a target
        # even while its (fast, occluded) track is COASTING, so the fleet keeps
        # chasing its predicted position. The pellet goes to the front.
        targets = [t for t in self.tracker.tracks
                   if t.status == tracking.CONFIRMED or self.threat_flags.get(t.id, False)]
        targets.sort(key=lambda t: not self.threat_flags.get(t.id, False))

        for t in targets:
            tx, ty = float(t.x[0]), float(t.x[1])
            pellet = self.threat_flags.get(t.id, False)
            if t.id in self.assignments:                      # already being chased
                rid = self.assignments[t.id]
                lx, ly = self.last_goal.get(rid, (1e9, 1e9))
                if math.hypot(tx - lx, ty - ly) > RETARGET_DIST:
                    self.send_goal(rid, t.id, tx, ty)         # retarget the pursuer
                continue
            # normal coins go only to genuinely free robots that are close
            # enough to reach them before the track drops; the pellet is
            # auctioned first among ALL robots (any distance), so it can divert a
            # busy one — the fleet prioritizes it.
            if pellet:
                eligible = set(self.robots)
            else:
                eligible = {r for r in self.robots if r not in assigned
                            and self._reach(r, tx, ty) < worldmap.MAX_REACH}
            if not eligible:
                continue
            winner = self.auction_task(t.id, tx, ty, eligible, pellet=pellet)
            if winner is not None:
                assigned.add(winner)

    def _reach(self, rid: str, tx: float, ty: float) -> float:
        st = self.status.get(rid)
        if st is None:
            return float('inf')
        return math.hypot(st.x - tx, st.y - ty)

    def auction_task(self, track_id: int, tx: float, ty: float,
                     eligible, pellet: bool = False):
        bidders = []
        for rid in eligible:
            st = self.status.get(rid)
            if st is None:
                continue
            state = RobotStatus.LOST if rid in self.lost else st.state
            bidders.append(auction.Bidder(
                rid, st.x, st.y, state, st.battery, st.task_track_id))
        winner, bids = auction.run_auction(bidders, tx, ty, track_id)
        if winner is None:
            return None
        # the winner may have been chasing another coin — drop that assignment
        for tid, rid in list(self.assignments.items()):
            if rid == winner and tid != track_id:
                self.assignments.pop(tid)
                self.last_goal.pop(rid, None)
        pretty = ', '.join(f'{r}={b:.1f}' if b != float('inf') else f'{r}=x'
                           for r, b in bids.items())
        tag = 'PELLET ' if pellet else ''
        self.get_logger().info(
            f'AUCTION {tag}coin {track_id} @ ({tx:.0f},{ty:.0f}): {pretty} -> {winner}')
        self.assignments[track_id] = winner
        self.send_goal(winner, track_id, tx, ty)
        return winner

    def send_goal(self, robot_id: str, track_id: int, x: float, y: float):
        client = self.nav_clients[robot_id]
        if not client.server_is_ready():
            self.get_logger().warn(f'{robot_id}: action server not ready')
            return
        goal = NavigateToWaypoint.Goal()
        goal.x, goal.y = x, y
        client.send_goal_async(goal)
        self.last_goal[robot_id] = (x, y)

    # ------------------------------------------------------------- watchdog
    def watchdog_tick(self):
        now = time.monotonic()
        for rid in self.robots:
            seen = self.last_seen.get(rid)
            if seen is None:
                continue
            if now - seen > WATCHDOG_S and rid not in self.lost:
                self.lost.add(rid)
                self.get_logger().error(
                    f'COMMS LOST with {rid} '
                    f'(no heartbeat for {now - seen:.1f}s)')

    # ------------------------------------------------------------- outputs
    def publish_fleet(self):
        fs = FleetStatus()
        fs.header.stamp = self.get_clock().now().to_msg()
        fs.header.frame_id = 'map'
        for rid in self.robots:
            st = self.status.get(rid)
            if st is None:
                continue
            if rid in self.lost:
                st.state = RobotStatus.LOST
            fs.robots.append(st)
        self.fleet_pub.publish(fs)

    def publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        def base(ns, mtype):
            nonlocal mid
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = mtype
            m.pose.orientation.w = 1.0
            m.lifetime.nanosec = int(0.3e9)
            return m

        for t in self.tracker.tracks:
            x, y = float(t.x[0]), float(t.x[1])
            threat = self.threat_flags.get(t.id, False)
            coasting = t.status == tracking.COASTING
            if t.status == tracking.TENTATIVE:
                rgb, alpha = (0.55, 0.55, 0.60), 0.4      # unconfirmed sighting
            elif threat:
                rgb, alpha = (1.0, 0.20, 0.20), 0.5 if coasting else 1.0   # power pellet
            else:
                rgb, alpha = (1.0, 0.82, 0.0), 0.5 if coasting else 1.0    # a coin

            m = base('tracks', Marker.SPHERE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 1.4
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = *rgb, alpha
            arr.markers.append(m)

            # power pellet: a tall flashing "target designated" beam
            if threat and not coasting:
                flash = 0.55 + 0.45 * math.sin(time.monotonic() * 10.0)
                beam = base('pellet_beam', Marker.CYLINDER)
                beam.pose.position.x, beam.pose.position.y, beam.pose.position.z = x, y, 3.0
                beam.scale.x = beam.scale.y = 0.4
                beam.scale.z = 6.0
                beam.color.r, beam.color.g, beam.color.b, beam.color.a = 1.0, 0.2, 0.2, flash
                arr.markers.append(beam)

            # is the assigned chaser within capture range? -> "locked, about to bag it"
            rid_chase = self.assignments.get(t.id)
            near = False
            if rid_chase and rid_chase in self.status:
                stc = self.status[rid_chase]
                near = math.hypot(stc.x - x, stc.y - y) < worldmap.CAPTURE_R * 1.4
            if t.status == tracking.TENTATIVE:
                ell = (0.55, 0.55, 0.60)
            elif threat:
                ell = (1.0, 0.25, 0.25)
            elif near:
                ell = (0.15, 1.0, 0.35)             # LOCK — capture imminent
            else:
                ell = (1.0, 0.82, 0.0)

            # covariance ellipse (2-sigma) from the position block of P — the
            # fleet's "lock-on": it shrinks as certainty grows, floored to a
            # readable size, and turns green when a robot is about to capture.
            p2 = t.p[:2, :2]
            evals, evecs = np.linalg.eigh(p2)
            evals = np.maximum(evals, 1e-6)
            ang = math.atan2(evecs[1, 1], evecs[0, 1])
            # floor for readability, cap so a coasting "lost" coin doesn't balloon
            major = min(4.5, max(1.8, 4.0 * math.sqrt(evals[1])))
            minor = min(4.5, max(1.8, 4.0 * math.sqrt(evals[0])))
            e = base('cov', Marker.CYLINDER)
            e.pose.position.x, e.pose.position.y, e.pose.position.z = x, y, 0.12
            e.pose.orientation.z = math.sin(ang / 2)
            e.pose.orientation.w = math.cos(ang / 2)
            e.scale.x, e.scale.y, e.scale.z = major, minor, 0.06
            e.color.r, e.color.g, e.color.b, e.color.a = *ell, 0.45
            arr.markers.append(e)
            # bright outline ring around the ellipse
            ring = base('cov_ring', Marker.LINE_STRIP)
            ring.scale.x = 0.1
            ca, sa = math.cos(ang), math.sin(ang)
            ring.points = []
            for k in range(25):
                th = 2 * math.pi * k / 24
                ex, ey = (major / 2) * math.cos(th), (minor / 2) * math.sin(th)
                ring.points.append(Point(x=x + ex * ca - ey * sa,
                                         y=y + ex * sa + ey * ca, z=0.16))
            ring.color.r, ring.color.g, ring.color.b, ring.color.a = *ell, 0.95
            arr.markers.append(ring)

            # velocity vector
            vx, vy = float(t.x[2]), float(t.x[3])
            if math.hypot(vx, vy) > 0.15:
                v = base('vel', Marker.ARROW)
                v.points = [Point(x=x, y=y, z=1.4),
                            Point(x=x + vx, y=y + vy, z=1.4)]
                v.scale.x, v.scale.y = 0.07, 0.15
                v.color.r, v.color.g, v.color.b, v.color.a = *rgb, 0.9
                arr.markers.append(v)

            # label: shout about the pellet, whisper about a coasting (lost) coin
            label = None
            if threat:
                label = ('★ PELLET  p={:.2f}'.format(t.threat_prob), 0.95)
            elif coasting:
                label = ('coin — lost', 0.5)
            if label is not None:
                txt = base('track_labels', Marker.TEXT_VIEW_FACING)
                txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, 2.4
                txt.text = label[0]
                txt.scale.z = label[1]
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = *rgb, 1.0
                arr.markers.append(txt)

        # assignment lines (which robot is chasing which coin)
        for tid, rid in self.assignments.items():
            st = self.status.get(rid)
            tr = next((t for t in self.tracker.tracks if t.id == tid), None)
            if st is None or tr is None:
                continue
            line = base('assignments', Marker.LINE_LIST)
            line.points = [Point(x=st.x, y=st.y, z=0.6),
                           Point(x=float(tr.x[0]), y=float(tr.x[1]), z=0.6)]
            line.scale.x = 0.07
            col = worldmap.COLORS.get(rid, (0.9, 0.9, 0.9))
            line.color.r, line.color.g, line.color.b, line.color.a = *col, 0.85
            arr.markers.append(line)

        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = Hub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
