"""Market-based (auction) task allocation for the HUB.

When a threat track needs eyes on it, every robot submits a bid (cost);
the HUB awards the task to the lowest bidder. This is the single-item
sealed-bid flavor of market-based multi-robot task allocation (the same
family as CBBA/murdoch) — deliberately simple, easily explainable, and
strictly better than nearest-idle because the cost folds in battery and
current tasking, not just distance.

Pure python — no ROS imports — so it is unit-testable.
"""
import math
from dataclasses import dataclass

# RobotStatus.state values (mirrors fleet_interfaces/RobotStatus)
IDLE, PATROL, INVESTIGATE, RTB, LOST = 0, 1, 2, 3, 4

INELIGIBLE = float('inf')
LOW_BATTERY_PENALTY = 20.0    # meters-equivalent, battery < 30%
RETASK_PENALTY = 15.0         # already investigating a *different* track


@dataclass
class Bidder:
    robot_id: str
    x: float
    y: float
    state: int
    battery: float
    task_track_id: int = 0


def bid(robot: Bidder, tx: float, ty: float, track_id: int,
        subsidy: float = 0.0) -> float:
    """One robot's cost to take the task. Lower wins.

    `subsidy` discounts the bid — used to make a high-value target (the power
    pellet) worth diverting even a busy robot to chase.
    """
    if robot.state in (RTB, LOST):
        return INELIGIBLE
    cost = math.hypot(robot.x - tx, robot.y - ty)
    if robot.battery < 30.0:
        cost += LOW_BATTERY_PENALTY
    if robot.state == INVESTIGATE and robot.task_track_id not in (0, track_id):
        cost += RETASK_PENALTY
    return cost - subsidy


def run_auction(robots: list[Bidder], tx: float, ty: float, track_id: int,
                subsidy: float = 0.0) -> tuple[str | None, dict[str, float]]:
    """Award the (tx, ty) chase task. Returns (winner_id, all_bids)."""
    bids = {r.robot_id: bid(r, tx, ty, track_id, subsidy) for r in robots}
    eligible = {rid: b for rid, b in bids.items() if b != INELIGIBLE}
    if not eligible:
        return None, bids
    winner = min(eligible, key=eligible.get)
    return winner, bids
