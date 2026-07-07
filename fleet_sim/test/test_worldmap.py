"""Unit tests for the shared world: occupancy + line-of-sight."""
from fleet_sim import worldmap


def test_buildings_are_occupied_streets_are_free():
    assert worldmap.is_occupied(14.0, 12.0)       # inside B1
    assert not worldmap.is_occupied(24.0, 20.0)   # central crossroads
    assert not worldmap.is_occupied(4.0, 4.0)     # sw corner street


def test_los_blocked_by_building():
    # B1 spans (8,8)-(20,16); looking straight through it
    assert not worldmap.line_of_sight(6.0, 12.0, 22.0, 12.0)


def test_los_clear_along_street():
    assert worldmap.line_of_sight(6.0, 20.0, 40.0, 20.0)   # central E-W street


def test_ray_distance_sees_wall():
    # from the street west of B1 looking east into its wall at x=8
    d = worldmap.ray_distance(5.0, 12.0, 0.0, max_range=6.0)
    assert 2.3 < d < 3.4
    # looking north along the open street: max range
    d2 = worldmap.ray_distance(4.0, 10.0, 1.5708, max_range=4.0)
    assert d2 == 4.0
