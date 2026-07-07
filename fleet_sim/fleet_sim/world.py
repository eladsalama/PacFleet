"""World node: publishes the arena occupancy grid + the neon maze markers.

The grid itself is defined in worldmap.py (shared by every node); this node
makes it visible two ways: a latched nav_msgs/OccupancyGrid (used for
line-of-sight math, kept for the "it's real ROS" story) and neon extruded
CUBE markers so the maze reads as a glowing 3D arcade board in RViz.
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from . import worldmap

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

WALL = (0.10, 0.32, 1.0)        # neon blue maze walls
WALL_CAP = (0.35, 0.85, 1.0)    # brighter cyan top edge
GROUND = (0.03, 0.04, 0.10)     # near-black arena floor


class WorldNode(Node):
    def __init__(self):
        super().__init__('world')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', LATCHED)
        self.publish_map()
        self.publish_buildings()
        self.get_logger().info(
            f'urban compound online: {worldmap.NX}x{worldmap.NY} cells @ '
            f'{worldmap.RESOLUTION} m, {len(worldmap.BUILDINGS)} buildings')

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = worldmap.RESOLUTION
        msg.info.width = worldmap.NX
        msg.info.height = worldmap.NY
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = worldmap.GRID.flatten().astype(int).tolist()
        self.map_pub.publish(msg)

    def publish_buildings(self):
        arr = MarkerArray()

        # arena floor: one big dark slab so the maze sits on a surface, not black
        floor = Marker()
        floor.header.frame_id = 'map'
        floor.ns = 'ground'
        floor.id = 0
        floor.type = Marker.CUBE
        floor.pose.position.x = worldmap.WIDTH / 2.0
        floor.pose.position.y = worldmap.HEIGHT / 2.0
        floor.pose.position.z = -0.05
        floor.pose.orientation.w = 1.0
        floor.scale.x = worldmap.WIDTH + 8.0
        floor.scale.y = worldmap.HEIGHT + 8.0
        floor.scale.z = 0.1
        floor.color.r, floor.color.g, floor.color.b, floor.color.a = *GROUND, 1.0
        arr.markers.append(floor)

        for i, (xmin, ymin, xmax, ymax) in enumerate(worldmap.BUILDINGS):
            h = 2.5 + (i % 3) * 1.1          # varied skyline
            cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'walls'
            m.id = i
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, h / 2
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = xmax - xmin, ymax - ymin, h
            m.color.r, m.color.g, m.color.b, m.color.a = *WALL, 1.0
            arr.markers.append(m)
            # bright neon cap along the top edge
            cap = Marker()
            cap.header.frame_id = 'map'
            cap.ns = 'wall_caps'
            cap.id = i
            cap.type = Marker.CUBE
            cap.pose.position.x, cap.pose.position.y, cap.pose.position.z = cx, cy, h + 0.06
            cap.pose.orientation.w = 1.0
            cap.scale.x, cap.scale.y, cap.scale.z = xmax - xmin + 0.15, ymax - ymin + 0.15, 0.2
            cap.color.r, cap.color.g, cap.color.b, cap.color.a = *WALL_CAP, 1.0
            arr.markers.append(cap)

        self._add_stations(arr)
        self.marker_pub.publish(arr)

    def _add_stations(self, arr: MarkerArray):
        """A recharging dock at each robot's home corner: a glowing pad, a
        charger cabinet with a beacon post, and a name label — all in the
        robot's colour so you can tell whose dock it is."""
        for i, rid in enumerate(worldmap.ROBOTS):
            hx, hy = worldmap.HOMES[rid]
            col = worldmap.COLORS[rid]

            pad = Marker()
            pad.header.frame_id = 'map'
            pad.ns = 'dock_pad'
            pad.id = i
            pad.type = Marker.CYLINDER
            pad.pose.position.x, pad.pose.position.y, pad.pose.position.z = hx, hy, 0.06
            pad.pose.orientation.w = 1.0
            pad.scale.x = pad.scale.y = 3.4
            pad.scale.z = 0.12
            pad.color.r, pad.color.g, pad.color.b, pad.color.a = *col, 0.45
            arr.markers.append(pad)

            cab = Marker()                       # charger cabinet
            cab.header.frame_id = 'map'
            cab.ns = 'dock_unit'
            cab.id = i
            cab.type = Marker.CUBE
            cab.pose.position.x, cab.pose.position.y, cab.pose.position.z = hx, hy, 0.6
            cab.pose.orientation.w = 1.0
            cab.scale.x, cab.scale.y, cab.scale.z = 0.9, 0.9, 1.2
            cab.color.r, cab.color.g, cab.color.b, cab.color.a = 0.12, 0.13, 0.16, 1.0
            arr.markers.append(cab)

            beacon = Marker()                    # glowing beacon post on top
            beacon.header.frame_id = 'map'
            beacon.ns = 'dock_beacon'
            beacon.id = i
            beacon.type = Marker.CYLINDER
            beacon.pose.position.x, beacon.pose.position.y, beacon.pose.position.z = hx, hy, 1.9
            beacon.pose.orientation.w = 1.0
            beacon.scale.x = beacon.scale.y = 0.16
            beacon.scale.z = 1.4
            beacon.color.r, beacon.color.g, beacon.color.b, beacon.color.a = *col, 1.0
            arr.markers.append(beacon)

            label = Marker()
            label.header.frame_id = 'map'
            label.ns = 'dock_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = hx, hy, 3.1
            label.pose.orientation.w = 1.0
            label.text = f'{rid.upper()} ⚡ DOCK'
            label.scale.z = 0.9
            label.color.r, label.color.g, label.color.b, label.color.a = *col, 1.0
            arr.markers.append(label)


def main():
    rclpy.init()
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
