"""Lab 12 — Phase 1: offline path planner.

Loads a world map (Lab 9 real-room or modified simulator), builds an
occupancy grid by point-in-polygon on cell centers (so half-cell walls
snap onto the obstacle side automatically), runs A* between each pair
of consecutive waypoints, and prints the (heading, distance) segments
that the onboard turn-go-turn primitive will consume.

Run from the notebooks/ folder:

    python lab12_plan.py

Outputs:
    * Console: a table of segments (heading in deg, distance in ft and m).
    * lab12_plan_<map>.png: visualization with obstacle cells drawn as
      square patches and the planned path overlaid.
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from astar import (
    Segment,
    astar,
    build_occupancy_grid,
    cell_to_world,
    cells_to_segments,
    polygon_lines,
    world_to_cell,
)

# ---------------------------------------------------------------------------
# Maps  — each is { 'outer': polygon, 'obstacles': [polygon, ...] }
# Polygons are sequences of (x, y) points in CCW order; the implied last
# edge wraps back to the first point.
# ---------------------------------------------------------------------------

# Map 1: Lab 9 real-room ground-truth measurements (feet).
MAP_LAB9 = {
    'outer': [
        ( 6.8,  4.5), ( 6.4, -4.3), ( 0.3, -4.3), ( 0.3, -2.5),
        (-0.7, -2.5), (-0.8, -4.5), (-5.9, -4.7), (-5.9,  0.6),
        (-2.2,  0.6), (-2.2,  4.8),
    ],
    'obstacles': [
        [( 2.7,  1.6), ( 4.8,  1.6), ( 4.8, -0.5), ( 2.7, -0.5)],
    ],
}

# Map 2: Simulator world (from world.yaml), modified per Lab 12 plan.
# Right wall pushed out to x=7 (more clearance for waypoints at x=5);
# inner obstacle is a 1 ft × 1 ft box centered at (3, 0).
MAP_SIM = {
    'outer': [
        (-5.5, -4.5), ( 6.5, -4.5), ( 6.5,  4.5), (-2.5,  4.5),
        (-2.5,  0.5), (-5.5,  0.5),
    ],
    'obstacles': [    
        [( 2.5, -0.5), ( 4.5, -0.5),
        ( 4.5, -0.5), ( 4.5,  1.5),
        ( 4.5,  1.5), ( 2.5,  1.5),
        ( 2.5,  1.5), ( 2.5, -0.5)],
    ],
}

# --- Switch which map the planner uses -------------------------------------
MAP_NAME = "sim"           # "sim" or "lab9"

_MAPS = {
    "sim":  (MAP_SIM,  (-6.0, 7.0), (-5.0, 5.0)), 
    "lab9": (MAP_LAB9, (-7.0, 7.0), (-5.0, 5.0)),
}
WORLD, X_RANGE, Y_RANGE = _MAPS[MAP_NAME]

# Lab 12 waypoint sequence (feet, 1 ft grid).
WAYPOINTS = [
    (-4, -3),  # start
    (-2, -1),
    ( 1, -1),
    ( 2, -3),
    ( 5, -3),
    ( 5, -2),
    ( 5,  3),
    ( 0,  3),
    ( 0,  0),  # end
]

# Grid setup
CELL_SIZE = 1.0           # ft. 1.0 matches world.yaml cell_size_x = 0.3048 m.
INFLATE   = 0             # 0 = polygon containment only; bump for safety margin.
FT_TO_M   = 0.3048        # robot operates in meters, but planner runs in feet.

# Robot footprint (18 cm × 10 cm chassis from the lab robot).
# Use the bounding circle so the robot is clear of every wall even while
# rotating in place between segments.
ROBOT_LENGTH_M = 0.18
ROBOT_WIDTH_M  = 0.10
ROBOT_RADIUS   = math.hypot(ROBOT_LENGTH_M, ROBOT_WIDTH_M) / 2 / FT_TO_M  # ~0.338 ft


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------

def plan_full_path():
    grid, origin = build_occupancy_grid(
        WORLD['outer'], WORLD['obstacles'],
        X_RANGE, Y_RANGE,
        cell_size=CELL_SIZE,
        inflate_cells=INFLATE,
        robot_radius=ROBOT_RADIUS,
    )

    all_segments: list[Segment] = []
    full_path_cells: list[tuple[int, int]] = []

    for i, (wa, wb) in enumerate(zip(WAYPOINTS, WAYPOINTS[1:])):
        sa = world_to_cell(wa, origin, CELL_SIZE)
        sb = world_to_cell(wb, origin, CELL_SIZE)
        cells = astar(grid, sa, sb)
        if not cells:
            print(f"!! no path {wa} -> {wb}")
            continue
        full_path_cells.extend(cells if i == 0 else cells[1:])

        segs = cells_to_segments(cells, origin, CELL_SIZE)
        print(f"\n=== {wa} -> {wb} ({len(segs)} segment(s)) ===")
        for k, s in enumerate(segs):
            print(
                f"  [{k}] heading {s.heading_deg:+7.2f}°,"
                f"  distance {s.distance:5.2f} ft  ({s.distance * FT_TO_M:.3f} m),"
                f"  {s.p_start} -> {s.p_end}"
            )
        all_segments.extend(segs)

    print(f"\nTotal segments to execute: {len(all_segments)}")
    return grid, origin, full_path_cells, all_segments


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def plot(grid, origin, full_path_cells, segments):
    fig, ax = plt.subplots(figsize=(11, 8))

    # Obstacle cells as actual squares (size = cell_size in data coordinates).
    occ_x, occ_y = np.where(grid == 1)
    for cx_idx, cy_idx in zip(occ_x, occ_y):
        cx, cy = cell_to_world((cx_idx, cy_idx), origin, CELL_SIZE)
        ax.add_patch(Rectangle(
            (cx - CELL_SIZE / 2, cy - CELL_SIZE / 2),
            CELL_SIZE, CELL_SIZE,
            facecolor='#cccccc', edgecolor='#999999', linewidth=0.4,
        ))

    # Original wall lines from polygons.
    for (p, q) in polygon_lines(WORLD['outer']):
        ax.plot([p[0], q[0]], [p[1], q[1]], 'k-', linewidth=2)
    for obs in WORLD['obstacles']:
        for (p, q) in polygon_lines(obs):
            ax.plot([p[0], q[0]], [p[1], q[1]], 'k-', linewidth=2)

    # Full A* cell path.
    if full_path_cells:
        xs = [cell_to_world(c, origin, CELL_SIZE)[0] for c in full_path_cells]
        ys = [cell_to_world(c, origin, CELL_SIZE)[1] for c in full_path_cells]
        ax.plot(xs, ys, color='tab:blue', linewidth=1.0, alpha=0.5,
                label='A* cell path')

    # Simplified segments (one arrow per segment).
    for s in segments:
        ax.annotate(
            '',
            xy=s.p_end, xytext=s.p_start,
            arrowprops=dict(arrowstyle='->', color='tab:blue', lw=1.8),
        )

    # Waypoints.
    wxs, wys = zip(*WAYPOINTS)
    ax.scatter(wxs, wys, c='red', s=140, marker='*', zorder=5, label='waypoints')
    for i, (x, y) in enumerate(WAYPOINTS):
        ax.annotate(f' {i}', (x, y), fontsize=11)

    # Robot bounding circle at the first waypoint (visual scale reference).
    x0, y0 = WAYPOINTS[0]
    ax.add_patch(Circle(
        (x0, y0), ROBOT_RADIUS,
        facecolor='none', edgecolor='tab:orange', linewidth=1.5,
        linestyle='--', label=f'robot r={ROBOT_RADIUS:.2f} ft',
    ))

    ax.set_aspect('equal')
    ax.set_xlim(X_RANGE[0] - 0.5, X_RANGE[1] + 0.5)
    ax.set_ylim(Y_RANGE[0] - 0.5, Y_RANGE[1] + 0.5)
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_xlabel('x (ft)')
    ax.set_ylabel('y (ft)')
    ax.legend(loc='upper left')
    ax.set_title(f'Lab 12 — A* over {MAP_NAME} map (cell={CELL_SIZE} ft, '
                 f'inflate={INFLATE})')
    plt.tight_layout()
    out = f'lab12_plan_{MAP_NAME}.png'
    plt.savefig(out, dpi=130)
    print(f"Saved {out}")


if __name__ == "__main__":
    grid, origin, path_cells, segments = plan_full_path()
    plot(grid, origin, path_cells, segments)
