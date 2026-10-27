+++
title = "Lab 12: Path Planning and Execution"
date = "2026-05-14"
+++

This lab combines path planning, orientation control, and localization to navigate through nine waypoints. I used A\* on the host computer to generate a path, then sent each straight segment to the Arduino over BLE. For the navigation runs, I used orientation PID for turning and heading correction, with time-based stopping for forward distance. I also tested periodic Bayes localization to correct the estimated position. The robot followed the overall route, but wall contacts and incorrect localization updates limited its accuracy.

# Path Planning

The waypoint coordinates are in feet:

`(-4,-3) → (-2,-1) → (1,-1) → (2,-3) → (5,-3) → (5,-2) → (5,3) → (0,3) → (0,0)`

{{ image(path="content/posts/lab12/path.png", alt="path", width=1200, class="center" )}}

I compared two maps: the simulator geometry and the measured map from Lab 9. The final mission notebook uses the simulator geometry for planning and `world.yaml` for localization. The Lab 9 map provides a separate planning comparison.

## A* and Path Smoothing

I converted each map into an occupancy grid. A cell is occupied when its center is outside the room, inside an obstacle, or within the robot radius of a wall. Using an 18 cm × 10 cm chassis gives a bounding radius of approximately 0.103 m. This accounts for the robot footprint, but adds no extra margin for execution error.

A\* uses eight neighbors, with costs of 1 for straight moves and $\sqrt{2}$ for diagonal moves. The heuristic is Euclidean distance to the goal. A\* expands the cell with the lowest $f(n) = g(n) + h(n)$, combining the distance already traveled with the estimated remaining distance. This favors short routes while accounting for obstacles. Diagonal moves are blocked when either adjacent orthogonal cell is occupied. After planning, `smooth_path` replaces intermediate turns with straight segments that pass the grid line-of-sight check.

```python
def plan_segments(start_ft, goal_ft):
    sa = world_to_cell(start_ft, origin, CELL_SIZE)
    sb = world_to_cell(goal_ft, origin, CELL_SIZE)
    cells = astar(grid, sa, sb)
    if not cells:
        raise RuntimeError(f"No path {start_ft} -> {goal_ft}")
    smoothed = smooth_path(cells, grid)
    return cells_to_segments(
        smoothed, origin, CELL_SIZE, simplify_first=False)
```

{{ image(path="content/posts/lab12/plan_sim.png", alt="Earlier A-star plan on the simulator map with 1 ft cells", width=1200, class="center" )}}

The simulator map has axis-aligned walls. In this earlier 1 ft plan, the path from `(1,-1)` to `(2,-3)` includes an intermediate turn.

{{ image(path="content/posts/lab12/plan_lab9.png", alt="Earlier A-star plan on the measured Lab 9 map with 1 ft cells", width=1200, class="center" )}}

The Lab 9 map has tilted walls and a different obstacle boundary. Its saved plan takes a wider route around the right side of the box. Both figures number the waypoints from 0 to 8.

## Grid Resolution

I later reduced the planning cell size from 1 ft to **0.5 ft**. The saved finer-grid plan uses a direct segment from `(1,-1)` to `(2,-3)`. The physical robot radius remains 0.103 m; changing cell size changes how the map is sampled.

{{ image(path="content/posts/lab12/plan_sim_small.png", alt="Simulator plan with 0.5 ft cells and a smoothed diagonal segment", width=1200, class="center" )}}

I also changed the localization grid in `world.yaml`:

```yaml
cell_size_x: 0.1524
cell_size_y: 0.1524
cell_size_a: 20
max_cells_x: 24
max_cells_y: 18
max_cells_a: 18
```

This gives 7,776 pose cells. The saved final notebook output confirms this grid through its uniform prior of `1/7776`. Planning and localization therefore use the same spatial resolution, although their cell centers differ. Localization results are converted to world coordinates before being rounded to a planning cell.

# Local Control

Each segment contains a heading and distance. The Arduino turns toward the heading, stabilizes for 200 ms, drives forward, and stops. It then waits 1 s to record coast-down before returning a completion message.

| Function | Method used in navigation |
| --- | --- |
| Turn to the segment heading | Orientation PID, gains `2.5, 1.2, 0.25` |
| Hold heading while driving | Same PID mixed into the left and right motor commands |
| Forward motion | Fixed 70% command with calibrated speed of 1.7 m/s |
| Primary distance stop | Elapsed drive time, `distance / calibrated speed` |
| Obstacle stop | Front ToF below 200 mm |
| Backup stop | Elapsed drive time above three times the expected duration |

During forward motion, I subtract the PID correction from one motor command and add it to the other. This adjusts heading while maintaining forward motion. It does not measure the remaining travel distance, so accurate heading alone cannot guarantee arrival.

The mission explicitly selects `SET_NAV_DIST_MODE = 0`. The KF still runs, but its distance estimate does not trigger the primary stop in this mode. For a 2 m segment, the expected drive time is $2/1.7 \approx 1.18$ s, excluding the turn, stabilization, and coast-down.

```cpp
float elapsed_s = (current_control_time - nav_phase_start_us) / 1.0e6f;
float expected_s = (nav_calib_speed_mps > 1e-3f)
                       ? nav_target_dist_m / nav_calib_speed_mps : 0.0f;
bool dist_done = nav_dist_traveled_m >= nav_target_dist_m;
bool time_done = elapsed_s >= NAV_TIME_SAFETY_MULT * expected_s;
bool primary_stop = nav_use_kf_dist ? dist_done : time_done;
const char *primary_reason = nav_use_kf_dist ? "dist" : "time";
bool backup_stop = elapsed_s >= 3.0f * expected_s;
bool safety_stop = tof2_dist > 0.0f && tof2_dist < nav_safety_tof_mm;

if (primary_stop || backup_stop || safety_stop)
{
    setMotors(0.0f, 0.0f);
    left_motor_pct = right_motor_pct = 0.0f;
    if (safety_stop)
        nav_stop_reason = "tof";
    else if (primary_stop)
        nav_stop_reason = primary_reason;
    else
        nav_stop_reason = "backup";
    nav_ack_pending = true;
    nav_state = NAV_TAIL;
    nav_phase_start_us = current_control_time;
}
```

## Distance Control Comparison

I also tested stopping from the change in Kalman-filtered front ToF distance. The filter is initialized before driving, predicts between measurements, and updates when a fresh valid ToF reading arrives. The stopping estimate is the absolute difference between the initial and current filtered range. This estimate depends on the sensor continuing to observe a suitable wall as the robot moves.

{{ image(path="content/posts/lab12/Nav_go.png", alt="Saved time-based and KF stopping tests with front ToF range and motor PWM", width=1200, class="center" )}}

These saved tests have different initial ranges: approximately 2.48 m for time mode and 1.50 m for KF mode. Their final ranges are approximately 1.27 m and 0.22 m. Therefore, the shared 1 m goal line does not provide a valid accuracy comparison. The dotted lines also start at the beginning of the segment, before forward driving begins. I selected time mode for navigation, but these traces alone do not establish its repeatability or isolate the cause of the KF error.

# Path Execution

The host plans from its current estimated position to the next waypoint, sends each segment, and waits for completion. A hop is the move between two waypoints and may contain several segments. If a segment stops on ToF, the host performs a rescue scan and replans to the same waypoint. This can recover from an interrupted move, but depends on the new position estimate being correct.

## Baseline Runs

The following runs compare the earlier 1 ft plan and the later 0.5 ft plan without periodic localization. The route is recognizable, but the wall contacts show that a valid planned path does not guarantee accurate execution.

**Earlier run with 1 ft planning cells**

[Watch the 1 ft run](https://youtube.com/shorts/OPj1jjvpnlI)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/OPj1jjvpnlI"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

**Later run with 0.5 ft planning cells**

[Watch the 0.5 ft run](https://youtube.com/shorts/KQI-t5f5REs)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/KQI-t5f5REs"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

## Periodic Localization

I reused the Lab 11 update step with a uniform prior. For each candidate pose, the filter compares the measured ranges with the expected map ranges. It multiplies their Gaussian likelihoods and normalizes the belief, then selects the most probable pose. A uniform prior allows any pose to win, including a distant incorrect match. Before scanning, the robot turns toward the estimated world heading of 0° and resets its yaw. The mapping FSM starts measurements after its first 20° turn. A 380° command allows measurements through 360° before the final turn. The host uses the first 18 valid ranges, applies the 69.85 mm sensor offset, and updates the position estimate.

After scanning, the host resets yaw again and assumes the robot faces −20° in the navigation frame. It does not use the Bayes heading estimate to correct this offset. Each observation call includes 22 s of waiting for scanning and 8 s for log transfer, before other overhead.

{{ image(path="content/posts/lab12/replanning.png", alt="replanning", width=1200, class="center" )}}

The live plotter shows targets, planned motion, and the localization estimate. Its green markers are commanded targets, rather than independent ground-truth measurements.

More frequent scans offer earlier position corrections but increase runtime. Across eight hops, intervals of two and three require four and three scheduled scans respectively, including the final hop; rescue scans can change the total.

**Trial 1: localization every two waypoint hops**

{{ image(path="content/posts/lab12/Trial1.png", alt="Trial 1 planned waypoints and stored position estimates", width=1200, class="center" )}}

[Watch localization Trial 1](https://youtube.com/shorts/ii9c4e570-Q)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/ii9c4e570-Q"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

**Trial 2: localization every three waypoint hops**

{{ image(path="content/posts/lab12/Trial2.png", alt="Trial 2 plans, rescue plans, and stored position estimates", width=1200, class="center" )}}

[Watch localization Trial 2](https://youtube.com/shorts/wdNE80N9eK0)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/wdNE80N9eK0"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

The blue points are stored position estimates. When localization is skipped, the code assigns the intended waypoint as the estimated position. Connecting these points does not show the measured physical trajectory. In Trial 2, green arrows show planned segments and orange dashes show rescue plans after a ToF stop.

# Discussion

The saved final notebook run illustrates the localization problem:

| Update | Intended goal in ft | Estimated position and framework angle | Peak probability |
| --- | --- | --- | --- |
| After hop 2 | `(1,-1)` | `(1.25,-1.75,110°)` | `1.0` |
| After hop 4 | `(5,-3)` | `(4.75,2.75,-170°)` | `0.9999734` |
| Rescue at hop 5 | `(5,-2)` | `(6.25,1.75,-170°)` | `0.9999849` |

Positions are in feet; the framework angle uses +x as 0° and counterclockwise positive. These are intended goals, not measured ground truth. After hop 4, the estimate is 5.76 ft from the goal, followed by two ToF stops during hop 5. A high posterior probability therefore does not establish physical accuracy.

IMU drift may contribute, but I did not isolate it from heading-reference or scan errors. A 5° heading error produces about 8.7 cm of lateral error over a 1 m straight segment. The much larger position jump requires checking the scan matching as well. The update uses range samples by index and ignores their measured bearings, while the post-scan heading is fixed at −20°. Both assumptions need validation before attributing the result to drift.

Forward motion is also sensitive to acceleration, battery condition, and coast-down because distance is controlled by time. At the configured 1.7 m/s, a 200 mm front range corresponds to only about 0.12 s of travel. The ToF stop cannot guarantee enough braking distance or detect every side contact. I would first test lower speed, calibrate short segments from rest, measure stopping distance, and add clearance beyond the chassis radius. I would also preserve the motion estimate between scans and reject implausible localization jumps before replanning.

# Conclusion

A\* produced useful paths on both maps, and the turn-and-drive controller executed the overall route. The main limitations were forward-distance accuracy and accepting localization results without validating them. Improving these two parts would make the existing planning approach more reliable.
