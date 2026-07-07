"""Deterministic urban-compound world shared by every node.

All nodes import this module directly (same constants everywhere), and the
`world` node additionally publishes the grid as nav_msgs/OccupancyGrid for
RViz. Pure numpy — no ROS imports — so it is unit-testable.

Coordinates are meters in the `map` frame, origin at the south-west corner.
"""
import heapq
import math
from collections import deque

import numpy as np

# ------------------------------------------------------------------ geometry
WIDTH = 60.0          # meters, x
HEIGHT = 40.0         # meters, y
RESOLUTION = 0.25     # meters / cell
NX = int(WIDTH / RESOLUTION)   # 240
NY = int(HEIGHT / RESOLUTION)  # 160

# Axis-aligned buildings: (xmin, ymin, xmax, ymax).
# Two rows of city blocks with a central E-W street at y=20 and
# N-S streets at x≈24 and x≈44.
BUILDINGS = [
    (8.0, 8.0, 20.0, 16.0),    # B1 south-west block
    (8.0, 24.0, 20.0, 32.0),   # B2 north-west block
    (28.0, 8.0, 40.0, 16.0),   # B3 south-east block
    (28.0, 24.0, 40.0, 32.0),  # B4 north-east block
    (46.0, 14.0, 54.0, 26.0),  # B5 far-east compound
]


def _build_grid() -> np.ndarray:
    """Occupancy grid, row-major [iy, ix]; 100 = occupied, 0 = free."""
    grid = np.zeros((NY, NX), dtype=np.int8)
    for xmin, ymin, xmax, ymax in BUILDINGS:
        ix0, ix1 = int(xmin / RESOLUTION), int(xmax / RESOLUTION)
        iy0, iy1 = int(ymin / RESOLUTION), int(ymax / RESOLUTION)
        grid[iy0:iy1, ix0:ix1] = 100
    # perimeter wall, one cell thick
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100
    return grid


GRID = _build_grid()


def cell_of(x: float, y: float) -> tuple[int, int]:
    ix = min(max(int(x / RESOLUTION), 0), NX - 1)
    iy = min(max(int(y / RESOLUTION), 0), NY - 1)
    return ix, iy


def is_occupied(x: float, y: float) -> bool:
    ix, iy = cell_of(x, y)
    return GRID[iy, ix] > 0


def line_of_sight(x0: float, y0: float, x1: float, y1: float) -> bool:
    """Bresenham traversal on the grid; False if any cell in between is occupied."""
    ix0, iy0 = cell_of(x0, y0)
    ix1, iy1 = cell_of(x1, y1)
    dx, dy = abs(ix1 - ix0), -abs(iy1 - iy0)
    sx = 1 if ix0 < ix1 else -1
    sy = 1 if iy0 < iy1 else -1
    err = dx + dy
    ix, iy = ix0, iy0
    while True:
        if GRID[iy, ix] > 0:
            return False
        if ix == ix1 and iy == iy1:
            return True
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            ix += sx
        if e2 <= dx:
            err += dx
            iy += sy


def ray_distance(x: float, y: float, theta: float, max_range: float = 4.0) -> float:
    """Distance to the first occupied cell along heading theta (for avoidance)."""
    step = RESOLUTION * 0.9
    d = step
    while d < max_range:
        if is_occupied(x + d * math.cos(theta), y + d * math.sin(theta)):
            return d
        d += step
    return max_range


# ------------------------------------------------------------------ path planning
# The robots plan around buildings with A* on an *inflated* copy of the grid
# (walls dilated by the robot radius) so paths keep clearance and never clip a
# corner — reactive whisker avoidance alone gets stuck in concave corners.
INFLATE_CELLS = 3        # ~0.75 m of clearance around every wall


def _inflate(occ: np.ndarray, cells: int) -> np.ndarray:
    inf = occ > 0
    for _ in range(cells):
        d = inf.copy()
        d[1:, :] |= inf[:-1, :]
        d[:-1, :] |= inf[1:, :]
        d[:, 1:] |= inf[:, :-1]
        d[:, :-1] |= inf[:, 1:]
        inf = d
    return inf


INFLATED = _inflate(GRID, INFLATE_CELLS)   # bool array, True = blocked-for-planning


def _cell_center(ix: int, iy: int) -> tuple[float, float]:
    return (ix + 0.5) * RESOLUTION, (iy + 0.5) * RESOLUTION


def _nearest_free_cell(ix: int, iy: int, max_ring: int = 60):
    """BFS out to the nearest cell that is free in the inflated grid."""
    if not INFLATED[iy, ix]:
        return ix, iy
    seen = {(ix, iy)}
    q = deque([(ix, iy)])
    while q:
        cx, cy = q.popleft()
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < NX and 0 <= ny < NY and (nx, ny) not in seen:
                if not INFLATED[ny, nx]:
                    return nx, ny
                seen.add((nx, ny))
                q.append((nx, ny))
        if len(seen) > (2 * max_ring) ** 2:
            break
    return None


def _los_cells(a: tuple[int, int], b: tuple[int, int]) -> bool:
    """True if the straight line a->b stays clear in the inflated grid."""
    ix0, iy0 = a
    ix1, iy1 = b
    dx, dy = abs(ix1 - ix0), -abs(iy1 - iy0)
    sx = 1 if ix0 < ix1 else -1
    sy = 1 if iy0 < iy1 else -1
    err = dx + dy
    ix, iy = ix0, iy0
    while True:
        if INFLATED[iy, ix]:
            return False
        if ix == ix1 and iy == iy1:
            return True
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            ix += sx
        if e2 <= dx:
            err += dx
            iy += sy


_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1),
              (-1, -1), (-1, 1), (1, -1), (1, 1)]


def _astar(start: tuple[int, int], goal: tuple[int, int]):
    openh = [(0.0, start)]
    came: dict = {}
    g = {start: 0.0}
    gx, gy = goal
    while openh:
        _, cur = heapq.heappop(openh)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        cx, cy = cur
        base = g[cur]
        for dx, dy in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < NX and 0 <= ny < NY) or INFLATED[ny, nx]:
                continue
            ng = base + (1.4142 if dx and dy else 1.0)
            nxt = (nx, ny)
            if ng < g.get(nxt, 1e18):
                g[nxt] = ng
                came[nxt] = cur
                heapq.heappush(openh, (ng + math.hypot(gx - nx, gy - ny), nxt))
    return None


def plan_path(start: tuple[float, float],
              goal: tuple[float, float]) -> list[tuple[float, float]]:
    """A* around the buildings. Returns sparse world-frame waypoints (excluding
    the start), string-pulled for straight runs, or [] if unreachable."""
    s = _nearest_free_cell(*cell_of(*start))
    g_cell = _nearest_free_cell(*cell_of(*goal))
    if s is None or g_cell is None:
        return []
    cells = _astar(s, g_cell)
    if not cells:
        return []
    # string-pull: keep only cells where a straight shortcut would clip a wall
    pulled = [cells[0]]
    anchor = 0
    for i in range(1, len(cells) - 1):
        if not _los_cells(cells[anchor], cells[i + 1]):
            pulled.append(cells[i])
            anchor = i
    pulled.append(cells[-1])
    pts = [_cell_center(ix, iy) for ix, iy in pulled]
    # aim at the true goal on the last leg if it isn't buried in a wall
    if not INFLATED[g_cell[1], g_cell[0]] and cell_of(*goal) == g_cell:
        pts[-1] = goal
    return pts[1:] if len(pts) > 1 else pts


# ------------------------------------------------------------------ the fleet
# Three collector robots that hunt coins through the maze.
ROBOTS = ['chomp', 'dash', 'nibble']

COLORS = {                      # RGB 0..1, neon arcade palette (URDF + markers)
    'chomp': (1.00, 0.50, 0.05),   # neon orange
    'dash': (0.10, 0.90, 1.00),    # neon cyan
    'nibble': (1.00, 0.20, 0.85),  # neon magenta
}

HOMES = {                       # recharge / rally points (maze corners)
    'chomp': (3.0, 3.0),
    'dash': (57.0, 3.0),
    'nibble': (57.0, 37.0),
}

SPAWNS = {                      # start pose x, y, yaw
    'chomp': (4.0, 6.0, 0.0),
    'dash': (44.0, 4.0, math.pi / 2),
    'nibble': (30.0, 20.0, 0.0),
}

# Search loops along the corridors (waypoint lists, cycled forever) that each
# robot roams while it has no coin assigned.
PATROL_ROUTES = {
    'chomp': [(4.0, 4.0), (24.0, 4.0), (24.0, 20.0), (24.0, 36.0), (4.0, 36.0), (4.0, 20.0)],
    'dash': [(26.0, 4.0), (44.0, 4.0), (44.0, 20.0), (44.0, 36.0), (26.0, 36.0), (24.0, 20.0)],
    'nibble': [(24.0, 20.0), (44.0, 20.0), (56.0, 20.0), (56.0, 30.0), (44.0, 20.0)],
}

# ------------------------------------------------------------------ coins / game
# Coins drift slowly along the corridors; the fleet hunts them. A coin is
# captured when a robot is close AND the coordinator is confident about it
# (its Kalman track is CONFIRMED — the "lock-on"). Occasionally a coin becomes
# a power pellet: it sprints and zigzags (cutting behind walls, which exercises
# occlusion -> track coasting -> reacquisition) and is worth far more points.
COIN_COUNT = 5             # coins live on the board at once
COIN_SPEED = 0.6           # m/s while wandering
CAPTURE_R = 2.4            # m: a robot this close can bag a locked-on coin
CAPTURE_ASSOC_R = 2.6      # m: how near a CONFIRMED track must be to count as lock-on
MAX_REACH = 15.0           # m: don't send a robot after a coin farther than this — it
                           # would lose sight of it (track drops) before arriving.
                           # The power pellet is exempt: the fleet chases it anywhere.
CAPTURE_FILL_RATE = 0.4    # capture-bar progress/sec per robot at full certainty
CAPTURE_DECAY = 0.3        # progress/sec lost when no robot is on the coin
COIN_VALUE = 10
PELLET_VALUE = 50

# power pellet (berserk) parameters. Speed is below robot V_MAX (1.4) so the
# fleet CAN run it down — but the heavy weave keeps it erratic (the classifier
# flags it on cross-track deviation) and makes it dart, so some still escape.
PELLET_INTERVAL = 24.0     # s between pellet events
PELLET_SPEED = 1.25        # m/s sprint (robots do 1.4)
PELLET_ZIGZAG = 0.9        # rad weave amplitude
PELLET_LIFETIME = 20.0     # s before it "escapes" back to a normal coin


def is_clear(x: float, y: float, clearance: float = 1.1) -> bool:
    """True if (x, y) and a small cross around it are all free — used to spawn
    coins in open corridors rather than jammed against a wall."""
    if is_occupied(x, y):
        return False
    for dx, dy in ((clearance, 0), (-clearance, 0), (0, clearance), (0, -clearance)):
        if is_occupied(x + dx, y + dy):
            return False
    return True


def random_free_xy(rng, margin: float = 2.5):
    """A random open point in the maze (rng is a random.Random)."""
    for _ in range(300):
        x = rng.uniform(margin, WIDTH - margin)
        y = rng.uniform(margin, HEIGHT - margin)
        if is_clear(x, y):
            return x, y
    return WIDTH / 2.0, HEIGHT / 2.0


# ------------------------------------------------------------------ sensor
SENSOR_RANGE = 12.0        # m
SENSOR_FOV = math.radians(110.0)
SENSOR_NOISE_STD = 0.35    # m, gaussian on measured position
P_MISS = 0.10              # dropped detection probability inside FOV
