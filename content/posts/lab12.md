+++
title = "Lab 12: Path Planning and Execution"
date = "2026-05-14"
+++

This final lab chains the planning, perception, and control primitives from earlier labs into an end-to-end mission through 9 waypoints. I pushed as much logic as possible offboard: the host Python runs A\* with line-of-sight smoothing and the Lab 11 Bayes update, while the Arduino executes one `(heading, distance)` turn-go-turn segment at a time. The two sides communicate over BLE through a new `MODE_NAV_SEG` state machine and a 4-field `D|S|F|Y` ack.

# Segment Navigation

The atomic motion primitive is a single turn-go-turn segment: rotate to a world-frame heading, then drive forward for a commanded distance. Both arguments arrive over BLE as `SET_NAV_TARGET heading_deg|distance_m|seg_id`, followed by `START_RECORD`. The Arduino computes nothing about world position — every segment is parameterised in its own (heading, length) frame, and the host owns the world model.

## Commands & State Machine

Four new BLE commands plus a new `MODE_NAV_SEG` value for `SET_MODE`:

- `SET_NAV_TARGET heading|dist|seg_id` — stores the next segment
- `SET_NAV_CALIB pwm|speed_mps` — sets `nav_go_pwm` (70%) and the calibrated forward speed used by the time-based stop
- `SET_NAV_DIST_MODE 0|1` — 0 = time-based stop, 1 = KF-integrated stop
- `RESET_YAW` — zero the relative-yaw reference so the host can switch between local and world frames

The on-board FSM `NAV_IDLE → NAV_TURN → NAV_STABILIZE → NAV_GO → NAV_TAIL → NAV_DONE` consumes one segment per `START_RECORD`. `NAV_STABILIZE` (200 ms) lets the IMU and ToF settle before the open-loop drive; `NAV_TAIL` (1 s post-stop) keeps the controller alive so the coast-down still appears in the log. `NAV_DONE` emits the ack `D: seg | S: stop_reason | F: final_tof_mm | Y: final_yaw_deg`, which tells the host both that the segment finished and *why* — `dist`, `time`, `tof`, or `backup`.

## Feedback Control

### Orientation Control

For `NAV_TURN` I reused the Lab 6 orientation PID directly ($K_p = 2.5, K_i = 1.2, K_d = 0.25$), exiting on $|err| < 3°$. The same controller already produced clean 20° steps in Lab 9 mapping, and a single segment only needs one rotation so cumulative drift is negligible.

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

For `NAV_GO` I overlay the orientation PID onto the open-loop forward command from Lab 9 so the robot keeps the commanded heading while driving straight. The mixed left/right outputs are clipped jointly so the heading correction always wins over the forward bias.

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

The interesting design choice for `NAV_GO` is *when to stop*. I implemented two modes and compared them on a fixed test.

#### TOF-value with KL-filter

The Lab 7/8 Kalman filter is seeded with the front ToF reading on entry to `NAV_GO`, then runs `kfPredict` every tick on the open-loop motor command and `kfUpdate` only when a fresh ToF reading arrives. Traveled distance is `|kf_pos_start − kf_pos_now|`, so the robot stops once the integrated travel ≥ target.

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

The alternative is to multiply the commanded distance by a calibrated `nav_calib_speed_mps`. After a few open-floor runs I locked in 1.7 m/s at 70% PWM, giving `elapsed_s ≈ 1.76` for a 2 m segment (best fine-tuning).

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

The test: start 3 m from the wall, command heading 0° and distance 2 m, the car should stop around 1 m from the wall.

Result:
{{ image(path="content/posts/lab12/Nav_go.png", alt="Time based vs KF Filter", width=1200, class="center" )}}

Time mode landed within ~10 cm of the 1 m mark across repeated runs. The KF mode overshot most trials: `kfPredict` does not capture the speed ramp-up at the start of `NAV_GO`, so the integrated distance lags the true travel and the stop fires late. The KF also drifts whenever the front ToF goes invalid (no target ahead, or off-axis return), which the time-based stop never sees.

So I chose **time-based control as the primary**, with the KF path running underneath as a 3× expected-time backup against a stuck integrator.


# Path Execution

Goal: navigate through these 9 waypoints in feet, with (-4, -3) as the start and (0, 0) as the end:

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


The host runs the full mission. For each consecutive pair of waypoints it calls A\*, smooths the result, and walks the segment list one at a time — each segment becomes a `SET_NAV_TARGET` + `START_RECORD` + `await NAV_DONE` round trip over BLE.

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

The occupancy grid is built by a point-in-polygon test on cell centers (`build_occupancy_grid`), then inflated by the robot's bounding-circle radius $\sqrt{0.18^2 + 0.10^2}/2 \approx 0.10$ m (≈ 0.338 ft) so the chassis stays clear of every wall during in-place rotation between segments. Standard 8-connected A\* with the Euclidean heuristic gives a cell path, but 8-connected output only emits 45°-multiple headings, which forces a zig-zag through any (dx, dy) ratio that isn't 1:0 or 1:1.

So I run `smooth_path` on top: a greedy line-of-sight string-puller that replaces zig-zags with the longest straight runs whose line is still obstacle-free. The resulting segments can have arbitrary heading (e.g. $\mathrm{atan2}(1, -2) \approx 153.4°$), and one smoothed move can replace 2–3 of the raw 45° zig-zags.

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

I planned the same 9 waypoints over two maps: the "sim" map from `world.yaml` (inner box around (3.5, 0.5), U-pillar near (0, -3.5)) and the "lab9" map I derived from the merged Lab 9 scatter plot.

{{ image(path="content/posts/lab12/plan_sim.png", alt="plan_sim", width=1200, class="center" )}}

{{ image(path="content/posts/lab12/plan_lab9.png", alt="plan_lab9", width=1200, class="center" )}}

The lab9 plan is noticeably more zig-zagged. The lab9 walls are tilted by a few degrees relative to integer-foot axes — Lab 9's ToF + IMU drift didn't produce perfectly orthogonal scans — so the inflated grid eats more cells along diagonals and `smooth_path` finds fewer long line-of-sight shortcuts. The sim plan, with axis-aligned walls, collapses most hops to one or two segments.

 
## Trial with Cell Size = 1

With 1 ft cells every waypoint sits exactly on a cell center, but the planner can only avoid obstacles in 1 ft increments. A\* takes the long way around waypoint 4 → 5.

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

Halving the cell size to 0.5 ft doubles the grid resolution. Waypoints still align on cell centers (integer feet hit cell centers at any 1/N-ft resolution), but the inflated buffer around the inner box shrinks from 1 ft to 0.5 ft, and `smooth_path` can step in (2, 1) or (3, 1) cell ratios that were impossible before. Headings now include values like 22.5° and collapses the zigzag into a single diagonal segment.

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

In both pre-mapping trials the time-based control accumulated 1–2 ft of drift by waypoint 7. The main reson is battery sag: the calibrated 1.7 m/s drops to ~1.4 m/s on a tired battery pack, and friction asymmetry adds yaw drift over the course of the mission. Re-localising after every hop would fix the drift, but this process is too slow at ~25 seconds per scan. Therefore, I added a periodic Bayes update from Lab 11.

{{ image(path="content/posts/lab12/replanning.png", alt="replanning", width=1200, class="center" )}}


After every `LOCALIZE_EVERY` hops, the host runs `localize_once` in four phases:

1. **Pre-scan turn.** Send a zero-distance `NAV_SEG` with target heading 0° (world +y). `NAV_GO` exits on the first tick because `dist_done` is immediately true, but `NAV_TURN` still runs to completion — the robot ends up facing +y.
2. **Reset yaw.** The IMU yaw is zeroed so the subsequent scan starts at IMU 0 ↔ world 0, matching Lab 11's calibration condition exactly.
3. **Mapping scan + Bayes update.** The Arduino runs the Lab 9/11 mapping FSM (`SET_MAP_DEGREES = 380`, `SET_MODE = 5`), spins 380°, and streams the 18 stabilized ToF readings back over `SEND_LOG`. The host filters out the `-1` sentinels written between `MAP_MEASURE` steps, applies the 69.85 mm offset, converts mm → m, and feeds the result to `loc.update_step()` with a uniform prior so the framework chooses the most likely `(x, y, θ)` cell.
4. **Post-scan reset + offset bookkeeping.** The scan ended at IMU +380° = world −20° (because `INVERT_HEADING = True`). A second `RESET_YAW` zeros the IMU at the new heading and the host records `yaw_world_offset = -20°` so subsequent `world_to_tx` calls compensate correctly.

I used a 380° (not 360°) sweep because the mapping FSM exits a few degrees early on the last step due to PID tolerance, and the 20° overshoot guarantees we always cover a full revolution. The −20° offset is the systematic consequence of that overshoot under the `INVERT_HEADING` sign convention.



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

[Video Here](https://youtube.com/shorts/ReplaceMe)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/ReplaceMe"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

## Discussion

In the plotter, the blue points are the believed trajectory (one dot per hop), the grey dashes are the designed pathway, the green arrows are A\* subsegments from the initial plan, and orange dashes mark rescue re-plans. Both trials show the same recovery pattern: when the car overshoots a waypoint, the next belief lands beyond the goal and A\* turns the robot back; when the car stops short, the belief lands short and A\* generates a shorter remaining path that still hits the original waypoint.

However, the path execution with localization performs *worse* than the no-localization baseline. Three potential reasons:

- **The belief itself drifts.** As I noted in the Lab 11 conclusion, poses on the right side of the map (waypoints 5–7) are feature-poor and often snap to a neighbour cell. That injects a at least 0.3 m systematic error into the next plan rather than removing it.
- **The 360° mapping rotation accumulates yaw drift.** After two scans the world-to-IMU offset is off by ~5°, biasing every subsequent segment heading by the same amount. Trial 1 (scan every 2 hops) shows this clearly — the believed trajectory rotates clockwise relative to the designed path after waypoint 5.
- **Each scan drains the battery faster** than straight-line driving, so the calibrated 1.7 m/s degrades faster between hops than in the no-localization run.

Trial 2 (every 3 hops) is slightly better on the right side because there's one less scan, but it then loses the late-mission correction at waypoint 7 → 8 that Trial 1 still gets.

# Conclusion

The full pipeline — A\* + line-of-sight smoothing offboard, periodic Bayes update offboard, turn-go-turn FSM onboard — runs end to end and reaches every waypoint at least once across the trials. Best-case timing is **~30 s** for the full 8-hop mission without localization and **~110 s** with localization at every other hop. However, the system is fragile: drift in any one component (e.g. battery sag, ToF dropout, mapping yaw drift or neighbour-cell localisation error) has an immediate impact on the next plan. The localisation update was not enough to compensate for the open-loop drift in the right half of the map.

Hardest problem I faced in this lab is the world-frame to local-frame conversion when adding localization mid-mission. I originally planned to let the robot scan from whatever heading it happened to land at after the previous segment, but the angle transformation is messed up. So I forced a turn-to-+y before every scan and mirrored the Lab 11 calibration setup.

**Future work**: add a Bug 0 / Bug 2 wall-following fallback. The FSM already exposes three distinct stop reasons (`time`, `dist`, `tof`, `backup`), so the host can detect a ToF safety stop and switch to a wall-follow controller before re-planning. The relevant terminating logic is already in place in `NAV_GO`:

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
