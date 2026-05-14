"""A* path planning over an inflated occupancy grid for Lab 12.

Pipeline
--------
1. ``build_occupancy_grid`` rasterizes a *polygon-based* world description
   (one outer-boundary polygon defining free space + a list of obstacle
   polygons) onto a cell grid by testing each cell **center** for point-in-
   polygon containment. This is a directional-snap rasterizer: a wall at
   exactly half a cell from the nearest cell center is always placed on
   the obstacle side, because cells whose centers are inside the obstacle
   are marked and cells on the free side are not.
2. ``astar`` runs 8-connected A* with an Euclidean heuristic between two
   cells, returning the cell path.
3. ``cells_to_segments`` collapses co-linear runs of cells into long
   straight ``Segment`` objects for the onboard turn-go-turn primitive.

Cell coordinate convention
--------------------------
``origin`` is the **world-frame center of cell (0, 0)**. So
``cell_to_world(c)`` returns the cell's center and ``world_to_cell(p)``
returns the cell whose center is closest to world point ``p``.

Heading convention
------------------
Degrees from +y, clockwise positive — matches Lab 9's IMU yaw
(0° = facing +y, 90° = facing +x).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]
Polygon = Sequence[Point]
Cell = Tuple[int, int]


# ---------------------------------------------------------------------------
# Cell <-> world coordinate conversion
# ---------------------------------------------------------------------------

def world_to_cell(p: Point, origin: Point, cell_size: float) -> Cell:
    """Return the cell whose **center** is closest to world point ``p``."""
    return (
        int(round((p[0] - origin[0]) / cell_size)),
        int(round((p[1] - origin[1]) / cell_size)),
    )


def cell_to_world(c: Cell, origin: Point, cell_size: float) -> Point:
    """Return the world-frame **center** of cell ``c``."""
    return (origin[0] + c[0] * cell_size, origin[1] + c[1] * cell_size)


# ---------------------------------------------------------------------------
# Polygon utilities
# ---------------------------------------------------------------------------

def polygon_lines(poly: Polygon) -> List[Tuple[Point, Point]]:
    """Yield consecutive (p_i, p_{i+1}) edges of a closed polygon (wraps)."""
    n = len(poly)
    return [(poly[i], poly[(i + 1) % n]) for i in range(n)]


def point_in_polygon(p: Point, poly: Polygon) -> bool:
    """Ray-casting test. Returns True for strictly interior points;
    points exactly on an edge are treated as outside."""
    x, y = p
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


# ---------------------------------------------------------------------------
# Occupancy grid
# ---------------------------------------------------------------------------

def _inflate(grid: np.ndarray, n: int) -> np.ndarray:
    """Dilate occupied cells by ``n`` in Chebyshev distance."""
    if n <= 0:
        return grid.copy()
    out = grid.copy()
    for _ in range(n):
        shifted = np.zeros_like(out)
        shifted[1:, :]  |= out[:-1, :]
        shifted[:-1, :] |= out[1:, :]
        shifted[:, 1:]  |= out[:, :-1]
        shifted[:, :-1] |= out[:, 1:]
        out = out | shifted
    return out


def _point_to_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Euclidean distance from point ``p`` to line segment ``ab``."""
    px, py = p
    ax, ay = a
    bx, by = b
    abx, aby = bx - ax, by - ay
    L2 = abx * abx + aby * aby
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / L2))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def build_occupancy_grid(
    outer: Polygon,
    obstacles: Iterable[Polygon],
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    cell_size: float = 1.0,
    inflate_cells: int = 0,
    robot_radius: float = 0.0,
) -> Tuple[np.ndarray, Point]:
    """Build an occupancy grid using point-in-polygon on cell centers,
    with optional Minkowski-style inflation by the robot's footprint.

    Each cell is marked occupied (1) if any of:
      * its center is **outside** the ``outer`` polygon (off-map / behind wall),
      * its center is **inside** any of the ``obstacles`` polygons, or
      * its center is within ``robot_radius`` of any wall edge.

    The last condition keeps the robot's bounding circle (radius =
    ``sqrt(L² + W²) / 2`` of the chassis) clear of every wall during
    in-place rotation between segments. Pass 0 to disable.

    Because containment is tested at cell centers, walls that lie at a
    half-cell offset automatically place the cell on the obstacle side:
    e.g. a right wall at ``x = 7.0`` with cells centered at integer feet
    leaves the cell at ``x = 7`` outside (occupied) and the cell at
    ``x = 6`` inside (free) — there is no ambiguous "wall cell".

    Returns
    -------
    grid   : (W, H) uint8 ndarray, 1 = occupied
    origin : world coord of cell (0, 0) **center**
    """
    x0, x1 = x_range
    y0, y1 = y_range
    width  = int(math.ceil((x1 - x0) / cell_size)) + 1
    height = int(math.ceil((y1 - y0) / cell_size)) + 1
    origin = (x0, y0)
    grid = np.zeros((width, height), dtype=np.uint8)

    obstacles = list(obstacles)
    all_edges: List[Tuple[Point, Point]] = list(polygon_lines(outer))
    for obs in obstacles:
        all_edges.extend(polygon_lines(obs))

    for col in range(width):
        for row in range(height):
            c = cell_to_world((col, row), origin, cell_size)
            if not point_in_polygon(c, outer):
                grid[col, row] = 1
                continue
            blocked = False
            for obs in obstacles:
                if point_in_polygon(c, obs):
                    grid[col, row] = 1
                    blocked = True
                    break
            if blocked:
                continue
            if robot_radius > 0.0:
                for a, b in all_edges:
                    if _point_to_segment_distance(c, a, b) < robot_radius:
                        grid[col, row] = 1
                        break

    return _inflate(grid, inflate_cells), origin


# ---------------------------------------------------------------------------
# A*
# ---------------------------------------------------------------------------

_NEIGHBORS = (
    ( 1,  0, 1.0), (-1,  0, 1.0), ( 0,  1, 1.0), ( 0, -1, 1.0),
    ( 1,  1, math.sqrt(2)), ( 1, -1, math.sqrt(2)),
    (-1,  1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
)


def astar(grid: np.ndarray, start: Cell, goal: Cell) -> List[Cell]:
    """8-connected A* with Euclidean heuristic on a binary grid.

    Start and goal cells are forced passable so endpoints that land inside
    an inflation buffer (but not on a true wall) still get a plan.
    """
    grid = grid.copy()
    grid[start] = 0
    grid[goal]  = 0

    W, H = grid.shape
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
    pq: list[tuple[float, Cell]] = [(h(start), start)]
    closed: set[Cell] = set()

    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return list(reversed(path))
        closed.add(cur)
        cx, cy = cur
        for dx, dy, step in _NEIGHBORS:
            n = (cx + dx, cy + dy)
            nx, ny = n
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if grid[n]:
                continue
            # Block diagonal moves that would clip an obstacle corner.
            if dx != 0 and dy != 0 and (grid[cx + dx, cy] or grid[cx, cy + dy]):
                continue
            ng = g[cur] + step
            if ng < g.get(n, math.inf):
                g[n] = ng
                parent[n] = cur
                heapq.heappush(pq, (ng + h(n), n))
    return []


# ---------------------------------------------------------------------------
# Path → segments
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Segment:
    heading_deg: float   # 0° = +y, 90° = +x, clockwise positive
    distance: float      # in the same units as cell_size (Lab 9: feet)
    p_start: Point
    p_end: Point


def _direction(a: Cell, b: Cell) -> Tuple[int, int]:
    return (int(np.sign(b[0] - a[0])), int(np.sign(b[1] - a[1])))


def simplify(path: Sequence[Cell]) -> List[Cell]:
    """Drop interior cells that lie on a straight 8-connected run.

    This is intended for raw A* output where every move is one of the 8
    grid directions. It WILL break any-angle paths produced by
    ``smooth_path`` (which can step in arbitrary integer dx/dy ratios),
    because `_direction` only checks the sign of dx/dy. Use the
    ``simplify_first=False`` knob on ``cells_to_segments`` after
    smoothing.
    """
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    prev = _direction(path[0], path[1])
    for i in range(1, len(path) - 1):
        d = _direction(path[i], path[i + 1])
        if d != prev:
            out.append(path[i])
            prev = d
    out.append(path[-1])
    return out


def line_of_sight(
    a: Cell,
    b: Cell,
    grid: np.ndarray,
    samples_per_cell: int = 4,
) -> bool:
    """Return True if the straight line from cell ``a`` to cell ``b`` is
    obstacle-free in the binary grid.

    The line is sampled at sub-cell resolution (``samples_per_cell``
    samples per cell-unit of travel) so thin diagonals don't slip past
    corner-adjacent obstacles. Both endpoints must themselves be free.
    """
    ax, ay = a
    bx, by = b
    W, H = grid.shape
    if not (0 <= ax < W and 0 <= ay < H and 0 <= bx < W and 0 <= by < H):
        return False
    if grid[ax, ay] or grid[bx, by]:
        return False
    dx = bx - ax
    dy = by - ay
    dist = math.hypot(dx, dy)
    n_samples = max(int(math.ceil(dist * samples_per_cell)), 1)
    for i in range(1, n_samples + 1):
        t = i / n_samples
        x = ax + t * dx
        y = ay + t * dy
        cx, cy = int(round(x)), int(round(y))
        if not (0 <= cx < W and 0 <= cy < H):
            return False
        if grid[cx, cy]:
            return False
    return True


def smooth_path(path: Sequence[Cell], grid: np.ndarray) -> List[Cell]:
    """Greedy string-pulling on top of A* output.

    Replaces 8-connected zig-zags with the longest straight runs whose
    line is still obstacle-free in ``grid``. The result is an any-angle
    path: adjacent cells may differ by arbitrary integer ``(dx, dy)``,
    so headings are no longer constrained to multiples of 45°.

    Pass the resulting path to ``cells_to_segments(..., simplify_first=False)``
    — the 8-connected ``simplify`` will incorrectly collapse cells whose
    8-connected dx/dy signs happen to match.
    """
    if len(path) <= 2:
        return list(path)
    out: List[Cell] = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        # Walk j back until we have line-of-sight from path[i].
        while j > i + 1 and not line_of_sight(path[i], path[j], grid):
            j -= 1
        out.append(path[j])
        i = j
    return out


def cells_to_segments(
    path: Sequence[Cell],
    origin: Point,
    cell_size: float,
    simplify_first: bool = True,
) -> List[Segment]:
    """Convert a cell path into ``Segment``s.

    By default the 8-connected ``simplify`` pass runs first (the right
    thing for raw A* output). Pass ``simplify_first=False`` for paths
    that have already been smoothed by ``smooth_path`` — those use
    arbitrary integer dx/dy ratios and ``simplify`` will incorrectly
    merge non-collinear runs.
    """
    simp = simplify(path) if simplify_first else list(path)
    segs: List[Segment] = []
    for a, b in zip(simp, simp[1:]):
        pa = cell_to_world(a, origin, cell_size)
        pb = cell_to_world(b, origin, cell_size)
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        heading = math.degrees(math.atan2(dx, dy))   # 0 = +y, CW positive
        dist = math.hypot(dx, dy)
        segs.append(Segment(heading, dist, pa, pb))
    return segs
