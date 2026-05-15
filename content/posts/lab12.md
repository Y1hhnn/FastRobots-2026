+++
title = "Lab 12: Path Planning and Execution"
date = "2026-05-14"
+++

This final lab chains the planning, perception, and control primitives from earlier labs into an end-to-end mission through 9 waypoints. The host Python runs A\* with line-of-sight smoothing and the Lab 11 Bayes update; the Arduino executes one `(heading, distance)` segment at a time. They communicate over BLE via a new `MODE_NAV_SEG` FSM and a 4-field `D|S|F|Y` ack.

# System Architecture

Offboard (host Python) owns the world map, the Bayes filter, and A\*. Onboard (Arduino) owns the IMU/ToF and closes a control loop at ~200 Hz.

# Segment Navigation

The atomic motion primitive is one turn-go segment: rotate to a world-frame heading, then drive forward for a commanded distance. Arguments arrive as `SET_NAV_TARGET heading_deg|distance_m|seg_id`, followed by `START_RECORD`. The Arduino tracks no world position — the host owns the world model.

## Commands & State Machine

Four new BLE commands plus a `MODE_NAV_SEG` value for `SET_MODE`:

- `SET_NAV_TARGET heading|dist|seg_id` — store the next segment
- `SET_NAV_CALIB pwm|speed_mps` — set `nav_go_pwm` (70%) and the calibrated forward speed
- `SET_NAV_DIST_MODE 0|1` — 0 = time-based stop, 1 = KF-integrated stop
- `RESET_YAW` — zero the relative-yaw reference to switch between local and world frames

The FSM `NAV_IDLE → NAV_TURN → NAV_STABILIZE → NAV_GO → NAV_TAIL → NAV_DONE` consumes one segment per `START_RECORD`:

- `NAV_IDLE`: motors off, waiting for `START_RECORD`.
- `NAV_TURN`: spin in place with Lab 6 PID, exits on `|yaw err| < 3°`.
- `NAV_STABILIZE`: hold heading 200 ms so IMU/ToF settle, then seed the KF.
- `NAV_GO`: open-loop 70% PWM with PID trimming L/R. Exits on `primary_stop` (time/KF), `safety_stop` (ToF < 200 mm), or `backup_stop` (3× expected time).
- `NAV_TAIL`: motors off, controller still ticking for 1 s so coast-down is logged.
- `NAV_DONE`: emit ack `D: seg | S: reason | F: tof | Y: yaw` once. The reason (`dist`/`time`/`tof`/`backup`) drives the rescue logic.

## Feedback Control

### Orientation Control

`NAV_TURN` reuses the Lab 6 orientation PID directly ($K_p = 2.5, K_i = 1.2, K_d = 0.25$), exiting on $|err| < 3°$. The same controller produced clean 20° steps in Lab 9 mapping, so drift over a single rotation is negligible.

```cpp
case NAV_TURN:
{
    // Spin in place to the target heading using orient PID
    orient_pid.setpoint = nav_target_heading_deg;
    float angular = orient_pid.compute(orient_sensor, dt, true);
    applyAngularOutput(-angular);
    if (fabs(orient_pid.error_value) < NAV_TURN_TOL_DEG)
    {
        nav_state = NAV_STABILIZE;
        nav_phase_start_us = current_control_time;
    }
    break;
}
```

For `NAV_GO` I overlay the orientation PID onto the open-loop forward command so the robot holds heading while driving straight. Mixed L/R outputs are clipped jointly so the heading correction always wins over the forward bias.

```cpp
case NAV_GO:
{
    // Open-loop forward at the calibrated PWM, with orientation PID mixed into left/right to keep the heading constant
    float angular = orient_pid.compute(orient_sensor, dt, true);
    float left_raw = nav_go_pwm - angular;
    float right_raw = nav_go_pwm + angular;
    float max_raw = max(fabs(left_raw), fabs(right_raw));
    if (max_raw > 100.0f)
    {
        float scale = 100.0f / max_raw;
        left_raw *= scale;
        right_raw *= scale;
    }
    left_motor_pct = constrain(left_raw * MOTOR_SCALE, -100.0f, 100.0f);
    right_motor_pct = constrain(right_raw * MOTOR_SCALE, -100.0f, 100.0f);
    setMotors(left_motor_pct, right_motor_pct);

    ....
}
```

### Distance Contorl

The key design choice for `NAV_GO` is *when to stop*. I implemented two modes and compared them.

#### TOF-value with KL-filter

The Lab 7/8 KF is seeded with the front ToF on entry to `NAV_GO`, runs `kfPredict` every tick on the motor command, and `kfUpdate` only on fresh ToF. Traveled distance is `|kf_pos_start − kf_pos_now|`; stop when ≥ target.

```cpp
case NAV_STABILIZE:
{
    orient_pid.setpoint = nav_target_heading_deg;
    float angular = orient_pid.compute(orient_sensor, dt, true);
    applyAngularOutput(-angular);
    if (current_control_time - nav_phase_start_us > NAV_STABILIZE_US)
    {
        setMotors(0.0f, 0.0f);
        // Seed the Kalman filter with the current ToF reading
        float kf_init = (tof2_dist > 10.0f && tof2_dist < 6000.0f)
                            ? tof2_dist
                            : NAV_KF_INIT_FALLBACK_MM;
        dist_pid.sensor_mode = PIDController::KALMAN;
        dist_pid.resetKalman(kf_init);
        nav_kf_pos_start_mm = dist_pid.kfPosition();
        nav_dist_traveled_m = 0.0f;
        nav_state = NAV_GO;
        nav_phase_start_us = current_control_time;
    }
    break;
}
```


```cpp
case NAV_GO:
{
    // Open-loop forward at the calibrated PWM, with orientation PID
    //...
    // KF-Filter Control
    dist_pid.output_value = nav_go_pwm;
    dist_pid.kfPredict(dt);
    if (tof2_updated && tof2_dist > 10.0f && tof2_dist < 6000.0f)
    {
        dist_pid.kfUpdate(tof2_dist);
    }
    dist_pid.sensor_value = dist_pid.kfPosition();
    nav_dist_traveled_m =
        fabs(nav_kf_pos_start_mm - dist_pid.kfPosition()) / 1000.0f;
    bool dist_done = nav_dist_traveled_m >= nav_target_dist_m;
    
}
```
#### Time Elasped Control

The alternative is to stop after `distance / nav_calib_speed_mps` seconds. A tape-measured 3 m run at 70% PWM averaged 1.7 m/s across three trials (σ < 0.05 m/s), giving `elapsed_s ≈ 1.76` for a 2 m segment.

```cpp
case NAV_GO:
{
    float elapsed_s = (current_control_time - nav_phase_start_us) / 1.0e6f;
    float expected_s = (nav_calib_speed_mps > 1e-3f)
                            ? nav_target_dist_m / nav_calib_speed_mps
                            : 0.0f;
    bool time_done = elapsed_s >= NAV_TIME_SAFETY_MULT * expected_s;
}
```

#### Analysis

Test: start 3 m from a wall, command heading 0° and distance 2 m — the car should stop ~1 m from the wall.

{{ image(path="content/posts/lab12/Nav_go.png", alt="Time based vs KF Filter", width=1200, class="center" )}}

Time mode landed within ~10 cm of the 1 m mark. KF mode overshot most trials: `kfPredict` misses the speed ramp-up, so integrated distance lags true travel and the stop fires late. KF also drifts when the front ToF goes invalid. So **time-based is primary**, with KF as a 3× expected-time backup against a stuck integrator.


# Path Execution

Goal: hit these 9 waypoints (feet), from (-4, -3) to (0, 0):

1. (-4, -3)    <--start
2. (-2, -1)
3. (1, -1)
4. (2, -3)
5. (5, -3)
6. (5, -2)
7. (5, 3)
8. (0, 3)
9. (0, 0)      <--end

{{ image(path="content/posts/lab12/path.png", alt="path", width=1200, class="center" )}}

Each segment uses only the $(\delta_{rot1}, \delta_{trans})$ pair of the Lab 10 odometry model — `heading_deg` for $\delta_{rot1}$ (absolute world heading), `distance` for $\delta_{trans}$. $\delta_{rot2}$ is dropped because the next segment's `NAV_TURN` absorbs it.

For each pair of waypoints the host calls A\*, smooths the result, and walks the segment list — each becomes a `SET_NAV_TARGET` + `START_RECORD` + `await NAV_DONE` BLE round trip.

```python
async def send_segment(seg_id, heading_deg, dist_m, timeout_s=20.0, poll_s=0.05):
    last_ack.pop(seg_id, None)
    ble.send_command(CMD.SET_NAV_TARGET, f"{heading_deg}|{dist_m}|{seg_id}")
    await asyncio.sleep(0.10)
    ble.send_command(CMD.START_RECORD, "")
    t0 = time.time()
    while seg_id not in last_ack:
        if time.time() - t0 > timeout_s:
            return None
        await asyncio.sleep(poll_s)
    return last_ack[seg_id]
```


## A-star Wayplanning

The occupancy grid is built by point-in-polygon on cell centers, then inflated by the chassis bounding-circle radius $\sqrt{0.18^2 + 0.10^2}/2 \approx 0.10$ m so the robot stays clear of every wall during in-place rotation.

### Algorithm

A\* expands nodes by $f(n) = g(n) + h(n)$ — $g(n)$ is actual cost from start, $h(n)$ an admissible heuristic. I use edge cost $1$ for cardinals and $\sqrt 2$ for diagonals with Euclidean $h(n) = \sqrt{(n_x - g_x)^2 + (n_y - g_y)^2}$ (≤ any grid path, so admissible). Diagonals are blocked when either orthogonal neighbour is occupied, and start/goal are forced passable so endpoints inside the inflation buffer still plan.

```python
def astar(grid, start, goal):
    grid = grid.copy(); grid[start] = 0; grid[goal] = 0
    g, parent, closed = {start: 0.0}, {}, set()
    h  = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
    pq = [(h(start), start)]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed: continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]; path.append(cur)
            return list(reversed(path))
        closed.add(cur)
        for dx, dy, step in _NEIGHBORS:           # 4 cardinals + 4 diagonals
            n = (cur[0] + dx, cur[1] + dy)
            if not in_bounds(n) or grid[n]: continue
            if dx and dy and (grid[cur[0]+dx, cur[1]] or grid[cur[0], cur[1]+dy]):
                continue                          # block corner-clipping diagonals
            ng = g[cur] + step
            if ng < g.get(n, math.inf):
                g[n], parent[n] = ng, cur
                heapq.heappush(pq, (ng + h(n), n))
    return []
```

`plan_segments` wraps this with cell/world conversion and the smoothing pass described under Trial 2:

```python
def plan_segments(start_ft, goal_ft):
    sa = world_to_cell(start_ft, origin, CELL_SIZE)
    sb = world_to_cell(goal_ft,  origin, CELL_SIZE)
    cells = astar(grid, sa, sb)
    if not cells:
        raise RuntimeError(f"No path {start_ft} -> {goal_ft}")
    smoothed = smooth_path(cells, grid)
    return cells_to_segments(smoothed, origin, CELL_SIZE,
                             simplify_first=False)
```

I planned the same 9 waypoints over two maps: the "sim" map from `world.yaml` and the "lab9" map I derived from the merged Lab 9 scatter plot.

{{ image(path="content/posts/lab12/plan_sim.png", alt="plan_sim", width=1200, class="center" )}}

{{ image(path="content/posts/lab12/plan_lab9.png", alt="plan_lab9", width=1200, class="center" )}}

The lab9 plan is more zig-zagged because its walls are tilted a few degrees off the integer-foot axes, so the inflated grid eats more cells along diagonals and `smooth_path` finds fewer long shortcuts. The sim plan, with axis-aligned walls, collapses most hops to 1–2 segments.

 
## Trial with Cell Size = 1

With 1 ft cells, every waypoint sits on a cell center but obstacles can only be avoided in 1 ft steps. A\* takes the long way around waypoint 4 → 5.

[Video Here](https://youtube.com/shorts/OPj1jjvpnlI)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/OPj1jjvpnlI"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>


 
## Trial with Cell Size = 2

Halving cell size to 0.5 ft doubles grid resolution and shrinks the inflation buffer from 1 ft to 0.5 ft. Raw 8-connected A\* only emits 45°-multiple headings, so I run `smooth_path` on top — a greedy string-puller that replaces zig-zag runs with the longest straight segments still clear via `line_of_sight`. After smoothing, headings become arbitrary (e.g. $\mathrm{atan2}(1, -2) \approx 153.4°$) and one move can replace 2–3 raw 45° hops. With the finer grid, `smooth_path` steps in (2, 1) or (3, 1) cell ratios, and the zig-zag at hop 6 → 7 collapses into a single diagonal.

{{ image(path="content/posts/lab12/plan_sim_small.png", alt="plan_sim_small", width=1200, class="center" )}}

[Video Here](https://youtube.com/shorts/KQI-t5f5REs)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/KQI-t5f5REs"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>


# Localization

In both pre-mapping trials the time-based control accumulated 1–2 ft of drift by waypoint 7 — battery sag drops the calibrated 1.7 m/s to ~1.4 m/s on a tired pack, and friction asymmetry adds yaw drift. Re-localising every hop would fix this, but at ~25 s per scan it is too slow. So I added a periodic Bayes update from Lab 11.

{{ image(path="content/posts/lab12/replanning.png", alt="replanning", width=1200, class="center" )}}


After every `LOCALIZE_EVERY` hops, `localize_once` runs four phases:

1. **Pre-scan turn.** A zero-distance `NAV_SEG` with heading 0° rotates the robot to face world +y.
2. **Reset yaw.** `RESET_YAW` zeros the IMU so the scan starts at IMU 0 ↔ world 0.
3. **Scan + Bayes update.** The mapping FSM (`SET_MODE = 5`, `SET_MAP_DEGREES = 380`) spins and streams 18 ToF readings; the host applies the 69.85 mm offset and runs `loc.update_step()` with a uniform prior.
4. **Post-scan reset.** The scan ends at IMU +380° = world −20° (under `INVERT_HEADING = True`); a second `RESET_YAW` plus `yaw_world_offset = -20°` keeps `world_to_tx` correct.

The 380° (not 360°) sweep absorbs the PID tolerance on the last step; the −20° offset is its systematic consequence.



```python
is_loc_hop = (hop % LOCALIZE_EVERY == 0) or (hop == n_hops)
if is_loc_hop and not rescued_this_hop:
    x_ft, y_ft, _theta, new_offset = await localize_once(
        yaw_world_offset, seg_id_base=hop * 100 + 90)
    believed_ft           = (x_ft, y_ft)
    yaw_world_offset      = new_offset
    current_world_heading = new_offset
```

## Trial 1
Mapping after every two segment navigations.
{{ image(path="content/posts/lab12/Trial1.png", alt="Trial1", width=1200, class="center" )}}

[Video Here](https://youtube.com/shorts/ii9c4e570-Q)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/ii9c4e570-Q"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

## Trial2
Mapping after every three segment navigations.
{{ image(path="content/posts/lab12/Trial2.png", alt="Trial2", width=1200, class="center" )}}

[Video Here](https://youtube.com/shorts/wdNE80N9eK0)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/wdNE80N9eK0"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

## Discussion

In the plotter, blue points are the believed trajectory (one per hop), grey dashes the designed pathway, green arrows the initial A\* segments, and orange dashes the rescue re-plans. Both trials show the same recovery pattern: an overshoot drops the belief beyond the goal and A\* turns the robot back; an undershoot produces a shorter remaining path that still hits the waypoint.

Still, localization performs *worse* than the no-localization baseline. Three reasons:

- **Belief drift.** Waypoints 5–7 are feature-poor and snap to a neighbour cell, injecting ≥ 0.3 m systematic error into the next plan instead of removing it.
- **Scan yaw drift.** After two scans the world-to-IMU offset is off by ~5°, biasing every subsequent heading. Trial 1's believed trajectory rotates clockwise after waypoint 5 because of this.
- **Battery cost.** Each scan drains the pack faster than straight driving, so 1.7 m/s degrades faster between hops.

Trial 2 (every 3 hops) is slightly better on the right side with one fewer scan but loses the late correction at 7 → 8 that Trial 1 gets.

# Conclusion

The pipeline runs end-to-end and hits every waypoint at least once: **~30 s** without localization, **~110 s** with localization every other hop. But it's fragile — drift in any one component (battery sag, ToF dropout, scan yaw, neighbour-cell localisation) bleeds into the next plan, and the Bayes update was not enough to compensate for open-loop drift on the right half of the map.

Hardest problem: the world ↔ local frame conversion when adding mid-mission localization. I originally let the robot scan from whatever heading it landed at, but the angle transform broke. Forcing a turn-to-+y before every scan (mirroring Lab 11's calibration) fixed it.

**Future work**: a Bug 0/2 wall-following fallback. The FSM already exposes four stop reasons (`dist`/`time`/`tof`/`backup`), so the host can detect a ToF stop and switch to wall-follow before re-planning. The terminating logic is already in `NAV_GO`:

```cpp
    bool safety_stop = (tof2_dist > 0.0f) && (tof2_dist < nav_safety_tof_mm);

    bool primary_stop = nav_use_kf_dist ? dist_done : time_done;
    const char *primary_reason = nav_use_kf_dist ? "dist" : "time";
    bool backup_stop = (elapsed_s >= 3.0f * expected_s);

    if (primary_stop || backup_stop || safety_stop)
    {
        setMotors(0.0f, 0.0f);
        left_motor_pct = 0.0f;
        right_motor_pct = 0.0f;
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
    break;
```
