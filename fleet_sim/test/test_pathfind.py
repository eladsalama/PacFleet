"""Unit tests for A* path planning around the buildings."""
import math

from fleet_sim import worldmap


def _segment_clear(a, b):
    """No point sampled along a->b lands in an occupied cell."""
    steps = int(max(2, 4 * (abs(a[0] - b[0]) + abs(a[1] - b[1]))))
    for k in range(steps + 1):
        t = k / steps
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if worldmap.is_occupied(x, y):
            return False
    return True


def test_path_routes_around_a_building():
    # west street to east street, straight line would cut through building B1 (8,8)-(20,16)
    start, goal = (5.0, 12.0), (25.0, 12.0)
    assert not worldmap.line_of_sight(*start, *goal)   # blocked straight through
    path = worldmap.plan_path(start, goal)
    assert path, 'expected a route around the building'
    # every leg (from the robot, through the waypoints) stays out of walls
    prev = start
    for wp in path:
        assert _segment_clear(prev, wp)
        prev = wp
    assert math.dist(path[-1], goal) < 2.0


def test_path_along_open_street_is_short():
    path = worldmap.plan_path((6.0, 20.0), (40.0, 20.0))   # clear E-W street
    assert path and len(path) <= 3        # string-pulled to a near-straight run


def test_goal_inside_a_wall_snaps_out():
    path = worldmap.plan_path((5.0, 20.0), (14.0, 12.0))   # goal inside B1
    assert path                            # still returns a route to the nearest free cell
    assert not worldmap.is_occupied(*path[-1])


def test_reachable_target_returns_nonempty():
    path = worldmap.plan_path((4.0, 4.0), (56.0, 36.0))    # opposite corners
    assert path
    prev = (4.0, 4.0)
    for wp in path:
        assert _segment_clear(prev, wp)
        prev = wp
