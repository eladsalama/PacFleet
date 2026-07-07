"""Coins node — the PacFleet game master.

Owns the ground truth: coins that drift through the maze, the capture rules, the
score, and the power-pellet events. A coin is captured when a robot is close
AND the coordinator is confident about it — its Kalman track is CONFIRMED (the
"lock-on"). So sensing + fusion, not just proximity, wins the point: a coin the
fleet has lost behind a wall (track coasting) can't be bagged until reacquired.

Publishes:
  truth    (PoseArray)   ground-truth coin positions — robot sensors read this
  markers  (MarkerArray) gold coins, the flashing power pellet, capture bursts,
                         floating score pop-ups, and the scoreboard
Subscribes:
  /<robot>/status (RobotStatus) robot positions (proximity half of a capture)
  /hub/tracks     (Track)       track status + position (the lock-on half)
"""
import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from fleet_interfaces.msg import RobotStatus, Track

from . import worldmap

TICK = 0.1  # 10 Hz

GOLD = (1.0, 0.82, 0.0)
PELLET_A = (1.0, 0.95, 0.55)     # power-pellet flash endpoints
PELLET_B = (1.0, 0.35, 0.05)


class Coin:
    """One wandering coin (ground truth). May become a power pellet."""
    _next_id = 1

    def __init__(self, rng: random.Random):
        self.rng = rng
        self.id = Coin._next_id
        Coin._next_id += 1
        self.x, self.y = worldmap.random_free_xy(rng)
        self.tx, self.ty = worldmap.random_free_xy(rng)
        self.is_pellet = False
        self.pellet_until = 0.0
        self.zig = 0.0
        self.progress = 0.0          # capture-bar fill, 0..1

    def pick_target(self):
        self.tx, self.ty = worldmap.random_free_xy(self.rng)

    def make_pellet(self, now: float):
        self.is_pellet = True
        self.pellet_until = now + worldmap.PELLET_LIFETIME
        self.pick_target()

    def revert(self):
        self.is_pellet = False

    def speed(self) -> float:
        return worldmap.PELLET_SPEED if self.is_pellet else worldmap.COIN_SPEED

    def step(self, dt: float):
        dx, dy = self.tx - self.x, self.ty - self.y
        if math.hypot(dx, dy) < 0.6:
            self.pick_target()
            return
        heading = math.atan2(dy, dx)
        if self.is_pellet:
            self.zig += dt * 0.9
            heading += worldmap.PELLET_ZIGZAG * math.sin(self.zig * math.pi)
        step = self.speed() * dt
        nx, ny = self.x + step * math.cos(heading), self.y + step * math.sin(heading)
        if not worldmap.is_occupied(nx, ny):
            self.x, self.y = nx, ny
            return
        # blocked (wall or a zigzag swing): fall back to the straight line, else repick
        h = math.atan2(dy, dx)
        nx, ny = self.x + step * math.cos(h), self.y + step * math.sin(h)
        if not worldmap.is_occupied(nx, ny):
            self.x, self.y = nx, ny
        else:
            self.pick_target()


class Effect:
    """A short-lived capture flourish: an expanding ring + a rising score pop-up."""
    def __init__(self, mid: int, x: float, y: float, value: int, pellet: bool, born: float):
        self.mid = mid
        self.x, self.y = x, y
        self.value = value
        self.pellet = pellet
        self.born = born
        self.ttl = 1.1


class CoinsNode(Node):
    def __init__(self):
        super().__init__('coins')
        self.rng = random.Random(20260707)
        self.coins = [Coin(self.rng) for _ in range(worldmap.COIN_COUNT)]
        self.robots: dict[str, tuple[float, float]] = {}
        self.tracks: dict[int, tuple] = {}   # id -> (x, y, status, cov_trace, ts)
        self.track_assignee: dict[int, str] = {}                      # id->robot chasing it
        self.score = 0
        self.captures = {r: 0 for r in worldmap.ROBOTS}
        self.effects: list[Effect] = []
        self._fx_id = 6000
        self.t = 0.0
        self.next_pellet_t = worldmap.PELLET_INTERVAL

        self.truth_pub = self.create_publisher(PoseArray, 'truth', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)
        self.cap_pub = self.create_publisher(Int32, 'captured', 10)
        for r in worldmap.ROBOTS:
            self.create_subscription(
                RobotStatus, f'/{r}/status',
                lambda m, rid=r: self.on_status(rid, m), 10)
        self.create_subscription(Track, '/hub/tracks', self.on_track, 10)
        self.create_timer(TICK, self.tick)
        self.get_logger().info(
            f'{len(self.coins)} coins in play; power pellet every '
            f'~{worldmap.PELLET_INTERVAL:.0f}s')

    # -------------------------------------------------------------- ingestion
    def on_status(self, rid: str, msg: RobotStatus):
        self.robots[rid] = (msg.x, msg.y)

    def on_track(self, msg: Track):
        cov_trace = msg.covariance[0] + msg.covariance[5]   # position variance (x+y)
        self.tracks[msg.track_id] = (
            msg.position.x, msg.position.y, msg.status, cov_trace, time.monotonic())
        self.track_assignee[msg.track_id] = msg.assigned_to

    # -------------------------------------------------------------- game loop
    def tick(self):
        self.t += TICK
        now = time.monotonic()
        self.tracks = {k: v for k, v in self.tracks.items() if now - v[4] < 1.5}
        self.track_assignee = {k: v for k, v in self.track_assignee.items()
                               if k in self.tracks}

        self.maybe_spawn_pellet(now)

        # A coin's capture bar fills while robots are on it and the fleet is
        # locked on. It fills faster with higher certainty and MORE robots
        # (swarming the pellet pays off); it drains when abandoned. Full -> caught.
        captured: list[tuple[int, str, int]] = []
        for i, coin in enumerate(self.coins):
            if coin.is_pellet and now >= coin.pellet_until:
                coin.revert()
                self.get_logger().info('power pellet escaped')
            coin.step(TICK)
            tid, certainty = self.locked_track(coin)
            n_near = sum(1 for (rx, ry) in self.robots.values()
                         if math.hypot(rx - coin.x, ry - coin.y) < worldmap.CAPTURE_R)
            if tid is not None and n_near >= 1:
                coin.progress += worldmap.CAPTURE_FILL_RATE * n_near * certainty * TICK
                if coin.progress >= 1.0:
                    captured.append((i, self.credit_robot(coin, tid), tid))
            else:
                coin.progress = max(0.0, coin.progress - worldmap.CAPTURE_DECAY * TICK)
        for i, rid, tid in captured:
            self.capture(i, rid, tid, now)

        self.publish_truth()
        self.publish_markers(now)

    def maybe_spawn_pellet(self, now: float):
        if self.t < self.next_pellet_t:
            return
        self.next_pellet_t = self.t + worldmap.PELLET_INTERVAL
        if any(c.is_pellet for c in self.coins):
            return
        coin = self.rng.choice(self.coins)
        coin.make_pellet(now)
        self.get_logger().warn(
            f'POWER PELLET is loose (coin drifting fast + weaving) '
            f'— worth +{worldmap.PELLET_VALUE}')

    def locked_track(self, coin: Coin) -> tuple[int | None, float]:
        """Nearest CONFIRMED track sitting on this coin -> the coordinator has
        lock-on. Returns (track id, certainty in 0.3..1 from the track's
        covariance), or (None, 0) if the fleet isn't confident about this coin."""
        best, best_d, best_cov = None, worldmap.CAPTURE_ASSOC_R, 1.0
        for tid, (tx, ty, status, cov_trace, _ts) in self.tracks.items():
            if status != Track.CONFIRMED:
                continue
            d = math.hypot(tx - coin.x, ty - coin.y)
            if d < best_d:
                best, best_d, best_cov = tid, d, cov_trace
        if best is None:
            return None, 0.0
        certainty = max(0.3, min(1.0, 1.2 / (best_cov + 0.4)))
        return best, certainty

    def credit_robot(self, coin: Coin, tid: int) -> str:
        """Who gets the point: the assigned pursuer if it's on the coin, else the
        nearest robot in range."""
        rid = self.track_assignee.get(tid, '')
        if rid in self.robots:
            rx, ry = self.robots[rid]
            if math.hypot(rx - coin.x, ry - coin.y) < worldmap.CAPTURE_R:
                return rid
        best, best_d = rid, worldmap.CAPTURE_R
        for r, (rx, ry) in self.robots.items():
            d = math.hypot(rx - coin.x, ry - coin.y)
            if d < best_d:
                best, best_d = r, d
        return best or next(iter(self.robots), '')

    def capture(self, idx: int, rid: str, tid: int, now: float):
        coin = self.coins[idx]
        value = worldmap.PELLET_VALUE if coin.is_pellet else worldmap.COIN_VALUE
        self.score += value
        self.captures[rid] = self.captures.get(rid, 0) + 1
        kind = 'POWER PELLET' if coin.is_pellet else 'coin'
        self.get_logger().info(
            f'{rid.upper()} captured {kind}  +{value}  (score {self.score})')
        self.cap_pub.publish(Int32(data=int(tid)))    # tell the HUB to free the pursuer
        self.effects.append(Effect(self._fx_id, coin.x, coin.y, value,
                                   coin.is_pellet, now))
        self._fx_id += 10
        self.coins[idx] = Coin(self.rng)   # respawn a fresh coin elsewhere

    # -------------------------------------------------------------- outputs
    def publish_truth(self):
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        for c in self.coins:
            p = Pose()
            p.position.x, p.position.y = c.x, c.y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.truth_pub.publish(pa)

    def publish_markers(self, now: float):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, c in enumerate(self.coins):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'coins'
            m.id = i
            m.pose.orientation.w = 1.0
            if c.is_pellet:
                flash = 0.5 + 0.5 * math.sin(now * 12.0)
                m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = c.x, c.y, 1.0
                m.scale.x = m.scale.y = m.scale.z = 1.7
                m.color.r = PELLET_A[0] * flash + PELLET_B[0] * (1 - flash)
                m.color.g = PELLET_A[1] * flash + PELLET_B[1] * (1 - flash)
                m.color.b = PELLET_A[2] * flash + PELLET_B[2] * (1 - flash)
                m.color.a = 1.0
            else:
                m.type = Marker.SPHERE            # a glowing gold orb (reads from any angle)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = c.x, c.y, 0.9
                m.scale.x = m.scale.y = m.scale.z = 1.4
                m.color.r, m.color.g, m.color.b, m.color.a = *GOLD, 1.0
            arr.markers.append(m)

            # capture progress bar (only while it's actually filling)
            if c.progress > 0.02:
                arr.markers.extend(self._capture_bar(i, c, stamp))
            else:
                arr.markers.append(self._delete(i, 'capbar'))
                arr.markers.append(self._delete(100 + i, 'capbar'))

        # capture flourish: a brief white pop + a big expanding shock ring + a
        # large rising "+N" — so a capture is unmistakable
        still: list[Effect] = []
        for e in self.effects:
            age = now - e.born
            if age > e.ttl:
                for off in (0, 1, 2):
                    arr.markers.append(self._delete(e.mid + off))
                continue
            still.append(e)
            frac = age / e.ttl
            ring_col = (1.0, 0.55, 0.1) if e.pellet else (0.35, 1.0, 0.45)

            flash = Marker()
            flash.header.frame_id = 'map'
            flash.header.stamp = stamp
            flash.ns = 'fx'
            flash.id = e.mid + 2
            flash.type = Marker.SPHERE
            flash.pose.position.x, flash.pose.position.y, flash.pose.position.z = e.x, e.y, 1.0
            flash.pose.orientation.w = 1.0
            flash.scale.x = flash.scale.y = flash.scale.z = 2.0 + frac * 6.0
            flash.color.r = flash.color.g = flash.color.b = 1.0
            flash.color.a = max(0.0, 0.9 * (1 - frac * 2.2))   # very brief bright pop
            arr.markers.append(flash)

            ring = Marker()
            ring.header.frame_id = 'map'
            ring.header.stamp = stamp
            ring.ns = 'fx'
            ring.id = e.mid
            ring.type = Marker.CYLINDER
            ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = e.x, e.y, 0.15
            ring.pose.orientation.w = 1.0
            ring.scale.x = ring.scale.y = 1.5 + frac * (11.0 if e.pellet else 8.0)
            ring.scale.z = 0.08
            ring.color.r, ring.color.g, ring.color.b = ring_col
            ring.color.a = max(0.0, 0.9 * (1 - frac))
            arr.markers.append(ring)

            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = stamp
            txt.ns = 'fx'
            txt.id = e.mid + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y = e.x, e.y
            txt.pose.position.z = 2.0 + frac * 3.5
            txt.pose.orientation.w = 1.0
            txt.text = f'+{e.value}'
            txt.scale.z = 1.9 if e.pellet else 1.4
            txt.color.r, txt.color.g, txt.color.b = ring_col
            txt.color.a = max(0.0, 1 - frac)
            arr.markers.append(txt)
        self.effects = still

        arr.markers.append(self.scoreboard(stamp))
        self.marker_pub.publish(arr)

    def scoreboard(self, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = 'scoreboard'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.x = worldmap.WIDTH / 2.0
        m.pose.position.y = worldmap.HEIGHT + 3.0
        m.pose.position.z = 5.0
        m.pose.orientation.w = 1.0
        tally = '   '.join(f'{r.upper()} {self.captures.get(r, 0)}'
                           for r in worldmap.ROBOTS)
        m.text = f'P A C F L E E T\nSCORE  {self.score:06d}\n{tally}'
        m.scale.z = 2.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.95, 0.3, 1.0
        return m

    def _delete(self, mid: int, ns: str = 'fx') -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        return m

    def _capture_bar(self, i: int, c: Coin, stamp) -> list[Marker]:
        p = max(0.0, min(1.0, c.progress))
        bg = Marker()
        bg.header.frame_id = 'map'
        bg.header.stamp = stamp
        bg.ns = 'capbar'
        bg.id = i
        bg.type = Marker.CUBE
        bg.pose.position.x, bg.pose.position.y, bg.pose.position.z = c.x, c.y, 2.7
        bg.pose.orientation.w = 1.0
        bg.scale.x, bg.scale.y, bg.scale.z = 2.6, 0.4, 0.06
        bg.color.r, bg.color.g, bg.color.b, bg.color.a = 0.1, 0.1, 0.12, 0.85

        fill = Marker()
        fill.header.frame_id = 'map'
        fill.header.stamp = stamp
        fill.ns = 'capbar'
        fill.id = 100 + i
        fill.type = Marker.CUBE
        width = 2.4 * p
        fill.pose.position.x = c.x - 1.2 + width / 2.0     # grow from the left
        fill.pose.position.y = c.y
        fill.pose.position.z = 2.72
        fill.pose.orientation.w = 1.0
        fill.scale.x, fill.scale.y, fill.scale.z = max(0.001, width), 0.28, 0.1
        fill.color.r = 0.2 + 0.6 * p        # green -> bright as it fills
        fill.color.g = 1.0
        fill.color.b = 0.2 + 0.5 * p
        fill.color.a = 1.0
        return [bg, fill]


def main():
    rclpy.init()
    node = CoinsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
