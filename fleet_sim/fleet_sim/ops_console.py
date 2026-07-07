"""Ops console — clickable robot rings.

Publishes one interactive "ring" per robot (ROS 2 interactive_markers) that
follows it around the arena. Clicking a ring with RViz's Interact tool emits
feedback on /ops/console/feedback, which the docked FleetPanel uses to focus
that robot. The live stats themselves live in the panel now.
"""
import rclpy
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)

from fleet_interfaces.msg import RobotStatus

from . import worldmap


class OpsConsole(Node):
    def __init__(self):
        super().__init__('ops_console')
        self.declare_parameter('robots', worldmap.ROBOTS)
        self.robots = list(self.get_parameter('robots').value)
        self.status: dict[str, RobotStatus] = {}

        self.server = InteractiveMarkerServer(self, 'console')
        for r in self.robots:
            self.create_subscription(
                RobotStatus, f'/{r}/status',
                lambda m, rid=r: self.on_status(rid, m), 10)
            self._insert(r)
        self.server.applyChanges()

        self.create_timer(0.2, self.follow)      # keep the rings on the robots (5 Hz)
        self.get_logger().info('ops console online — click a robot ring to focus it')

    def on_status(self, rid: str, msg: RobotStatus):
        self.status[rid] = msg

    def _pose(self, rid: str) -> Pose:
        p = Pose()
        st = self.status.get(rid)
        if st is not None:
            p.position.x, p.position.y = st.x, st.y
        p.orientation.w = 1.0
        return p

    def _make(self, rid: str) -> InteractiveMarker:
        col = worldmap.COLORS.get(rid, (1.0, 1.0, 1.0))
        im = InteractiveMarker()
        im.header.frame_id = 'map'
        im.name = rid
        im.description = rid.upper()          # always-on name label above the robot
        im.scale = 2.0
        im.pose = self._pose(rid)

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctrl.always_visible = True
        ring = Marker()
        ring.type = Marker.CYLINDER
        ring.scale.x = ring.scale.y = 2.6
        ring.scale.z = 0.12
        ring.pose.position.z = 0.07
        ring.pose.orientation.w = 1.0
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = *col, 0.6
        ctrl.markers.append(ring)
        im.controls.append(ctrl)
        return im

    def _insert(self, rid: str):
        self.server.insert(
            self._make(rid), feedback_callback=self.on_click,
            feedback_type=InteractiveMarkerFeedback.BUTTON_CLICK)

    def on_click(self, feedback):
        self.get_logger().info(f'focus: {feedback.marker_name}')

    def follow(self):
        moved = False
        for rid in self.robots:
            if rid in self.status:
                self.server.setPose(rid, self._pose(rid))
                moved = True
        if moved:
            self.server.applyChanges()


def main():
    rclpy.init()
    node = OpsConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
