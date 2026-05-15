+++
title = "Lab 12: Path Planning and Execution"
date = "2026-05-14"
+++

# Segment Navigation

Model & Math formula
Argument: Heading degree & distance

## Commands & State Machine

(Briefly Explain each state, newly added commands here)


## Feedback Control

### Orientation Control
(Use lab6 code for Nav_Turn)

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

(Use lab9 code for Nav_GO)
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

#### TOF-value with KL-filter

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

Best fine-tuning: `elapsed_s`=1.76
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
Start  3m from the wall, naviagtion with 0 heading degree and 2m, the car should stop around 1m from the wall.

Result:
{{ image(path="content/posts/lab12/Nav_go.png", alt="Time based vs KF Filter", width=1200, class="center" )}}

For time mode, we have a better control on the stop. For the KL-filter mode, its harder to control the drift distance and cause overshoot. (Or any other reasons..)

So choose time mode


# Path Execution

Goal: go through waypoints 1. (-4, -3)    <--start
2. (-2, -1)
3. (1, -1)
4. (2, -3)
5. (5, -3)
6. (5, -2)
7. (5, 3)
8. (0, 3)
9. (0, 0)      <--end

{{ image(path="content/posts/lab12/path.png", alt="path", width=1200, class="center" )}}


Host python sent the segement commands 

(Add slice of code from notebook/lab12_phase3.py)


## A-star Wayplanning

(Add slice of code from notebook/astar.py)

Explain the algorithm 

Use the map get from Lab9/Real map?

(Make the following image in the same row)
{{ image(path="content/posts/lab12/plan_sim.png", alt="plan_sim", width=1200, class="center" )}}

{{ image(path="content/posts/lab12/plan_lab9.png", alt="plan_lab9", width=1200, class="center" )}}

The lab9 path is more complex with zigzags becasue of the result get from tof is not accuarate
 
 
## Trial with Cell Size = 1
Go with the sim

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


 
## Trial with Cell Size = 2

state that how the cell of size will change (go 22.5)
{{ image(path="content/posts/lab12/plan_sim_small.png", alt="plan_sim_small", width=1200, class="center" )}}

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


# Localization

Both trials get serious drift in the later stae of mapping.The time elaspe is heavily depends on the battery state and changes when running in different distance (friction). 
Add mapping in Lab11. So it will periodicaly check its position using the bayes-probaility based localization and regenerate the path to the next goal.
Host python calculate the belief bad generate waypath online.

(Add slice of code from notebook/lab12_phase3.py)

{{ image(path="content/posts/lab12/replanning.png", alt="replanning", width=1200, class="center" )}}



## Trial 1
Mapping after every two segment navigation
{{ image(path="content/posts/lab12/Trial1.png", alt="Trial1", width=1200, class="center" )}}

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

## Trial2
Mapping after every three segment navigation
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

In the plotter, we can see the blue points (belived trajactory) drift from the grey dash line (designed pathway). The green lines are the real-time planned way. In the trial 1& 2 we can see if the car overshoot the next goal, it will replan to turn back. If the car is stop on the midway, it will drop the rest of trajectory and replan to reaching the new goal.

However, the path execution with localization perform even worse than the original one. Because the blief always drift (as state in the conclusion of Lab11), the car stop early or replan to the wrong way. Also, the rotation in mapping and more complex path makes the noise in time-elasped control even worse. 

# Conclusion

The structure works, but the localization and distance control all need to futher improve in order to achieve the accurate navigation. Improve including

Hardest Problem I faced in this lab: the world and local direction convertion (especially when adding the map). I was plan that the car behave the mapping with its current start direction but massed up. It now only turn to the +y direction before mapping, just as what I did in lab11.

Future work, add Bug0-2 algorithm, I have added terminating reason in Nav_GO.
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
```cpp