"""One simulated tactical UGV (diff-drive), instanced 3x under namespaces.

Owns everything a real UGV platform would expose to the HUB:
  - kinematics + odometry + tf (odom -> base_link) + joint_states (wheels spin)
  - an EO/IR-style FOV sensor: line-of-sight-checked, noisy detections
  - a NavigateToWaypoint action server (the HUB's tasking interface)
  - a behavior FSM: PATROL -> INVESTIGATE -> RTB (low battery) -> PATROL
  - a battery model and a 2 Hz RobotStatus heartbeat
  - manual override: any recent /<ns>/cmd_vel wins over autonomy (teleop demo)
"""
import math
import random
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from fleet_interfaces.action import NavigateToWaypoint
from fleet_interfaces.msg import Detection, RobotStatus

from . import worldmap

TICK = 0.05          # 20 Hz control loop
SENSOR_TICK = 0.2    # 5 Hz sensor
V_MAX = 1.4          # m/s
W_MAX = 2.2          # rad/s
WHEEL_R = 0.09
TRACK_W = 0.45
GOAL_TOL = 0.6       # m
BATTERY_DRAIN = 0.20     # %/s at full speed
BATTERY_RTB_AT = 20.0    # %
BATTERY_RESUME_AT = 95.0


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class UgvSim(Node):
    def __init__(self):
        super().__init__('ugv_sim')
        self.declare_parameter('robot_id', 'chomp')
        self.declare_parameter('battery0', 100.0)
        self.robot_id = self.get_parameter('robot_id').value
        self.battery = float(self.get_parameter('battery0').value)

        self.x, self.y, self.yaw = worldmap.SPAWNS[self.robot_id]
        self.home = worldmap.HOMES[self.robot_id]
        self.route = worldmap.PATROL_ROUTES[self.robot_id]
        self.route_i = 0
        self.color = worldmap.COLORS[self.robot_id]

        self.state = RobotStatus.PATROL
        self.task_track_id = 0
        self.v_cmd, self.w_cmd = 0.0, 0.0
        self.wheel_l_pos, self.wheel_r_pos = 0.0, 0.0
        self.manual_until = 0.0
        self.manual_cmd = (0.0, 0.0)
        self.nav_target: tuple[float, float] | None = None  # action goal target
        self.coins: list[tuple[float, float]] = []
        self.rng = random.Random(hash(self.robot_id) & 0xFFFF)
        self._stuck_ticks = 0            # consecutive ticks trying but not moving
        self._unstick_until = 0.0        # time.monotonic() deadline for the escape maneuver
        self._unstick_dir = 1.0          # which way to turn while backing out
        self.path: list[tuple[float, float]] = []   # A* waypoints to the current goal
        self.path_goal: tuple[float, float] | None = None
        self.path_time = 0.0

        cbg = ReentrantCallbackGroup()
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # --- I/O ---------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.det_pub = self.create_publisher(Detection, 'detections', sensor_qos)
        self.status_pub = self.create_publisher(RobotStatus, 'status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(PoseArray, '/coins/truth', self.on_truth,
                                 sensor_qos)

        self.tf = TransformBroadcaster(self)
        static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.robot_id}/odom'
        t.transform.rotation.w = 1.0
        static_tf.sendTransform(t)

        # --- tasking interface (the HUB talks to this) --------------------
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self.action_server = ActionServer(
            self, NavigateToWaypoint, 'navigate_to_waypoint',
            execute_callback=self.execute_nav,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            handle_accepted_callback=self.handle_accepted,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cbg)

        self.create_timer(TICK, self.control_tick, callback_group=cbg)
        self.create_timer(SENSOR_TICK, self.sensor_tick, callback_group=cbg)
        self.create_timer(0.2, self.publish_status, callback_group=cbg)
        self.get_logger().info(
            f'{self.robot_id} online at ({self.x:.1f}, {self.y:.1f}), '
            f'battery {self.battery:.0f}%')

    # ---------------------------------------------------------------- inputs
    def on_cmd_vel(self, msg: Twist):
        self.manual_cmd = (msg.linear.x, msg.angular.z)
        self.manual_until = time.monotonic() + 0.5

    def on_truth(self, msg: PoseArray):
        self.coins = [(p.position.x, p.position.y) for p in msg.poses]

    # ---------------------------------------------------------------- action
    def handle_accepted(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()      # preempt: newest goal wins
            self._goal_handle = goal_handle
        goal_handle.execute()

    def execute_nav(self, goal_handle):
        gx, gy = goal_handle.request.x, goal_handle.request.y
        self.nav_target = (gx, gy)
        if self.state != RobotStatus.RTB:
            self.state = RobotStatus.INVESTIGATE
        self.get_logger().info(f'{self.robot_id}: tasked to ({gx:.1f}, {gy:.1f})')
        fb = NavigateToWaypoint.Feedback()
        result = NavigateToWaypoint.Result()
        while rclpy.ok():
            dist = math.hypot(gx - self.x, gy - self.y)
            if not goal_handle.is_active:              # preempted by newer goal
                result.success = False
                result.final_distance = dist
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._nav_done()
                result.success = False
                result.final_distance = dist
                return result
            if dist < GOAL_TOL:
                goal_handle.succeed()
                self._nav_done()
                self.get_logger().info(f'{self.robot_id}: waypoint reached')
                result.success = True
                result.final_distance = dist
                return result
            fb.distance_remaining = dist
            goal_handle.publish_feedback(fb)
            time.sleep(0.2)
        result.success = False
        result.final_distance = 0.0
        return result

    def _nav_done(self):
        self.nav_target = None
        if self.state == RobotStatus.INVESTIGATE:
            self.state = RobotStatus.PATROL
            self.task_track_id = 0

    # ---------------------------------------------------------------- control
    def control_tick(self):
        # battery model drives the FSM harder than anything else
        drain = BATTERY_DRAIN * abs(self.v_cmd) / V_MAX + 0.01
        self.battery = max(0.0, self.battery - drain * TICK)
        if self.state != RobotStatus.RTB and self.battery < BATTERY_RTB_AT:
            self.state = RobotStatus.RTB
            self.get_logger().warn(
                f'{self.robot_id}: battery {self.battery:.0f}% -> RTB')

        now = time.monotonic()
        target = self.current_target()
        if now < self.manual_until:                      # operator override
            self.v_cmd, self.w_cmd = self.manual_cmd
        elif now < self._unstick_until:                  # escaping a corner: back out + turn
            self.v_cmd, self.w_cmd = -0.6, self._unstick_dir * W_MAX
        elif target is None:
            self.v_cmd, self.w_cmd = 0.0, 0.0
        else:
            wp, final = self.steer_target(target, now)
            self.v_cmd, self.w_cmd = self.go_to(wp, brake=final)

        # integrate diff-drive kinematics (with wall collision guard)
        px, py = self.x, self.y
        nx = self.x + self.v_cmd * math.cos(self.yaw) * TICK
        ny = self.y + self.v_cmd * math.sin(self.yaw) * TICK
        if not worldmap.is_occupied(nx, ny):
            self.x, self.y = nx, ny
        self.yaw = wrap(self.yaw + self.w_cmd * TICK)

        # stuck detector: trying to drive but pinned against geometry -> escape
        moved = math.hypot(self.x - px, self.y - py)
        if abs(self.v_cmd) > 0.2 and moved < 0.02 and now >= self._unstick_until:
            self._stuck_ticks += 1
            if self._stuck_ticks > 10:                   # ~0.5 s of no progress
                left = worldmap.ray_distance(self.x, self.y, self.yaw + 1.2, 3.0)
                right = worldmap.ray_distance(self.x, self.y, self.yaw - 1.2, 3.0)
                self._unstick_dir = 1.0 if left >= right else -1.0
                self._unstick_until = now + 0.8
                self._stuck_ticks = 0
        else:
            self._stuck_ticks = 0

        wl = (self.v_cmd - self.w_cmd * TRACK_W / 2) / WHEEL_R
        wr = (self.v_cmd + self.w_cmd * TRACK_W / 2) / WHEEL_R
        self.wheel_l_pos += wl * TICK
        self.wheel_r_pos += wr * TICK

        self.publish_odom()

    def current_target(self) -> tuple[float, float] | None:
        if self.state == RobotStatus.RTB:
            if math.hypot(self.home[0] - self.x, self.home[1] - self.y) < GOAL_TOL:
                self.battery = min(100.0, self.battery + 5.0 * TICK)  # recharging
                if self.battery >= BATTERY_RESUME_AT:
                    self.state = RobotStatus.PATROL
                    self.get_logger().info(
                        f'{self.robot_id}: recharged, resuming patrol')
                return None
            return self.home
        if self.nav_target is not None:
            return self.nav_target
        if self.state == RobotStatus.PATROL:
            tx, ty = self.route[self.route_i]
            if math.hypot(tx - self.x, ty - self.y) < GOAL_TOL:
                self.route_i = (self.route_i + 1) % len(self.route)
                tx, ty = self.route[self.route_i]
            return (tx, ty)
        return None

    def steer_target(self, goal: tuple[float, float], now: float):
        """Follow an A* path around the buildings. Returns (waypoint, is_final):
        the next waypoint to steer at, and whether it is the final goal (so we
        brake into it)."""
        gx, gy = goal
        stale = (not self.path or self.path_goal is None
                 or math.hypot(gx - self.path_goal[0], gy - self.path_goal[1]) > 2.0
                 or now - self.path_time > 1.0)
        if stale:
            self.path = worldmap.plan_path((self.x, self.y), goal)
            self.path_goal = goal
            self.path_time = now
        # pop waypoints we've reached
        while self.path and math.hypot(self.path[0][0] - self.x,
                                       self.path[0][1] - self.y) < 1.0:
            self.path.pop(0)
        if self.path:
            return self.path[0], False
        return goal, True                          # no path -> head straight in

    def go_to(self, target: tuple[float, float],
              brake: bool = True) -> tuple[float, float]:
        """P-controller on heading + ray-based wall avoidance. `brake` slows the
        approach — off for intermediate path waypoints, on for the final goal."""
        tx, ty = target
        dist = math.hypot(tx - self.x, ty - self.y)
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)

        # look ahead: front / left / right whiskers (wide, so corners read early)
        front = worldmap.ray_distance(self.x, self.y, self.yaw, 3.5)
        left = worldmap.ray_distance(self.x, self.y, self.yaw + 0.9, 2.5)
        right = worldmap.ray_distance(self.x, self.y, self.yaw - 0.9, 2.5)
        avoid = 0.0
        if front < 1.8:
            avoid = 2.2 if left >= right else -2.2
            if front < 0.7:                        # cornered: back out while turning away
                return -0.4, avoid

        w = max(-W_MAX, min(W_MAX, 2.5 * err + avoid))
        v = V_MAX if abs(err) < 0.6 else 0.35
        v = min(v, front)                          # never ram a wall
        if brake:
            v = min(v, 0.8 * dist + 0.1)           # ease into the final goal
        return v, w

    # ---------------------------------------------------------------- sensing
    def sensor_tick(self):
        """LOS-checked, noisy FOV sensor + its RViz cone/detection markers."""
        arr = MarkerArray()
        arr.markers.extend(self.fov_markers())
        det_id = 100
        for cx, cy in self.coins:
            dx, dy = cx - self.x, cy - self.y
            rng = math.hypot(dx, dy)
            bearing = wrap(math.atan2(dy, dx) - self.yaw)
            if rng > worldmap.SENSOR_RANGE or abs(bearing) > worldmap.SENSOR_FOV / 2:
                continue
            if not worldmap.line_of_sight(self.x, self.y, cx, cy):
                continue                              # occluded by a building
            if self.rng.random() < worldmap.P_MISS:
                continue                              # sensor dropout
            mx = cx + self.rng.gauss(0.0, worldmap.SENSOR_NOISE_STD)
            my = cy + self.rng.gauss(0.0, worldmap.SENSOR_NOISE_STD)

            d = Detection()
            d.header.stamp = self.get_clock().now().to_msg()
            d.header.frame_id = 'map'
            d.robot_id = self.robot_id
            d.position.x, d.position.y = mx, my
            d.range = rng
            d.bearing = bearing
            d.confidence = max(0.2, 1.0 - rng / worldmap.SENSOR_RANGE)
            self.det_pub.publish(d)

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = d.header.stamp
            m.ns = f'{self.robot_id}_detections'
            m.id = det_id
            det_id += 1
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = mx, my, 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.r, m.color.g, m.color.b = self.color
            m.color.a = 0.9
            m.lifetime.nanosec = int(0.4e9)
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def fov_markers(self) -> list[Marker]:
        """Sensor field-of-view as a translucent neon wedge + a bright outline.

        Both are in the moving base_link frame and stamped with time 0 so RViz
        uses the *latest* transform (an exact-time lookup intermittently fails
        while the robot moves and reddens the whole display)."""
        zero = rclpy.time.Time().to_msg()
        z = 0.35
        half = worldmap.SENSOR_FOV / 2
        n = 18
        arc = []
        for i in range(n + 1):
            a = -half + worldmap.SENSOR_FOV * i / n
            arc.append((worldmap.SENSOR_RANGE * math.cos(a),
                        worldmap.SENSOR_RANGE * math.sin(a)))
        apex = Point(x=0.0, y=0.0, z=z)

        wedge = Marker()
        wedge.header.frame_id = f'{self.robot_id}/base_link'
        wedge.header.stamp = zero
        wedge.ns = f'{self.robot_id}_fov_fill'
        wedge.id = 0
        wedge.type = Marker.TRIANGLE_LIST
        wedge.pose.orientation.w = 1.0
        wedge.scale.x = wedge.scale.y = wedge.scale.z = 1.0
        for i in range(n):
            wedge.points.append(apex)
            wedge.points.append(Point(x=arc[i][0], y=arc[i][1], z=z))
            wedge.points.append(Point(x=arc[i + 1][0], y=arc[i + 1][1], z=z))
        wedge.color.r, wedge.color.g, wedge.color.b = self.color
        wedge.color.a = 0.16

        outline = Marker()
        outline.header.frame_id = f'{self.robot_id}/base_link'
        outline.header.stamp = zero
        outline.ns = f'{self.robot_id}_fov'
        outline.id = 1
        outline.type = Marker.LINE_STRIP
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.05
        outline.color.r, outline.color.g, outline.color.b = self.color
        outline.color.a = 0.7
        outline.points = ([apex] + [Point(x=px, y=py, z=z) for px, py in arc]
                          + [apex])
        return [wedge, outline]

    # ---------------------------------------------------------------- outputs
    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = f'{self.robot_id}/odom'
        od.child_frame_id = f'{self.robot_id}/base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x = self.v_cmd
        od.twist.twist.angular.z = self.w_cmd
        self.odom_pub.publish(od)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = f'{self.robot_id}/odom'
        t.child_frame_id = f'{self.robot_id}/base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)

        js = JointState()
        js.header.stamp = now
        js.name = [f'{self.robot_id}/wheel_left_joint',
                   f'{self.robot_id}/wheel_right_joint']
        js.position = [self.wheel_l_pos, self.wheel_r_pos]
        self.joint_pub.publish(js)

    def publish_status(self):
        s = RobotStatus()
        s.robot_id = self.robot_id
        s.state = self.state
        s.x, s.y, s.yaw = self.x, self.y, self.yaw
        s.battery = self.battery
        s.speed = abs(self.v_cmd)
        s.task_track_id = self.task_track_id
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = UgvSim()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
