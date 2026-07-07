"""Unit tests for market-based task allocation."""
from fleet_sim.auction import (INVESTIGATE, PATROL, RTB, Bidder, run_auction)


def test_nearest_patroller_wins():
    robots = [Bidder('chomp', 0, 0, PATROL, 90),
              Bidder('dash', 50, 0, PATROL, 90)]
    winner, bids = run_auction(robots, 10, 0, track_id=1)
    assert winner == 'chomp'
    assert bids['chomp'] < bids['dash']


def test_rtb_robot_is_ineligible_even_if_closest():
    robots = [Bidder('chomp', 9, 0, RTB, 10),
              Bidder('dash', 40, 0, PATROL, 90)]
    winner, bids = run_auction(robots, 10, 0, track_id=1)
    assert winner == 'dash'
    assert bids['chomp'] == float('inf')


def test_low_battery_penalty_flips_close_call():
    # dash slightly farther but healthy; chomp closer but nearly flat
    robots = [Bidder('chomp', 5, 0, PATROL, 25),
              Bidder('dash', 12, 0, PATROL, 95)]
    winner, _ = run_auction(robots, 0, 0, track_id=1)
    assert winner == 'dash'


def test_busy_investigator_pays_retask_penalty():
    robots = [Bidder('chomp', 5, 0, INVESTIGATE, 90, task_track_id=7),
              Bidder('dash', 15, 0, PATROL, 90)]
    winner, _ = run_auction(robots, 0, 0, track_id=9)
    assert winner == 'dash'


def test_no_eligible_robots():
    robots = [Bidder('chomp', 0, 0, RTB, 5)]
    winner, _ = run_auction(robots, 10, 10, track_id=1)
    assert winner is None
