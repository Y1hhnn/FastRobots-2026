+++
title = "Lab 8: Stunts!"
date = "2026-04-08"
+++

# PID Controller

```cpp
struct PIDController
{
    // --- PID Gains, States, Tunings ---
    // ---------------------------------

    // --- Extrapolation States ---
    // ---------------------------------

    // --- Kalman Filter States ---
    // ---------------------------------

    // --- Sensor Estimation Mode ---
    enum SensorMode
    {
        DIRECT,
        EXTRAPOLATION,
        KALMAN
    };

    // Get the best sensor estimate based on sensor_mode.
    float getEstimate(float raw_measurement, bool has_new_measurement, float dt)
    {
        switch (sensor_mode)
        {
        case KALMAN:
            kfPredict(dt);
            if (has_new_measurement)
                kfUpdate(raw_measurement);
            return kfPosition();

        case EXTRAPOLATION:
            return getExtrapolated(micros());

        case DIRECT:
        default:
            return raw_measurement;
        }
    }

    // --- Helper Functions ---
    // ---------------------------------

    // Core PID compute
    float compute(float new_sensor, float dt, bool wrap_angle = false, bool use_kf_derivative = false)
    {
        float new_error;
        float delta_sensor;

        if (wrap_angle)
        {
            new_error = wrapAngle180(new_sensor - setpoint);
            delta_sensor = wrapAngle180(new_sensor - sensor_value);
        }
        else
        {
            new_error = new_sensor - setpoint;
            delta_sensor = new_sensor - sensor_value;
        }

        // Derivative
        if (use_kf_derivative)
        {
            // Use Kalman velocity estimate directly
            raw_derivative_value = kfVelocity();
            derivative_value = kfVelocity();
        }
        else
        {
            // Derivative on measurement (avoids derivative kick)
            raw_derivative_value = delta_sensor / dt;
            derivative_value = derivative_filter_alpha * raw_derivative_value +
                               (1.0f - derivative_filter_alpha) * derivative_value;
        }

        // Integral with anti-windup (conditional integration)
        raw_integral_value = integral_value + new_error * dt;
        float new_integral = constrain(raw_integral_value, -integral_limit, integral_limit);

        float unsat_output = kp * new_error + ki * new_integral + kd * derivative_value;
        bool saturated_high = unsat_output > output_limit;
        bool saturated_low = unsat_output < -output_limit;
        if ((!saturated_high && !saturated_low) ||
            (saturated_high && new_error < 0) ||
            (saturated_low && new_error > 0))
        {
            integral_value = new_integral;
        }

        output_value = constrain(kp * new_error + ki * integral_value + kd * derivative_value,
                                 -output_limit, output_limit);
        error_value = new_error;
        sensor_value = new_sensor;
        count++;

        return output_value;
    }
};

```

```cpp
// Distance PID — controls forward/backward via ToF
PIDController dist_pid;
// Orientation PID — controls turning via IMU yaw
PIDController orient_pid;
```

# Rush in Straight Line


```cpp
void runController(){
    // ---------------------------------
    case MODE_RUSH:
    {
        float estimate = dist_pid.getEstimate(tof2_dist, tof_updated, dt);
        dist_pid.sensor_value = estimate;
        dist_pid.output_value = 100.0f;
        float orient_sensor = getOrientationSensorValue();
        float angular = orient_pid.compute(orient_sensor, dt, true);

        float left_raw = 100.0f - angular;
        float right_raw = 100.0f + angular;
        float max_raw = max(left_raw, right_raw);
        if (max_raw > 100.0f)
        {
            float overshoot = max_raw - 100.0f;
            left_raw -= overshoot;
            right_raw -= overshoot;
        }
        left_motor_pct = constrain(left_raw * MOTOR_SCALE, -100.0f, 100.0f);
        right_motor_pct = constrain(right_raw * MOTOR_SCALE, -100.0f, 100.0f);
        setMotors(left_motor_pct, right_motor_pct);
        break;
    }
    // ---------------------------------
}
```

[Video Here](https://youtube.com/shorts/Pp7RTqsx2M0)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/Pp7RTqsx2M0"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab8/rush.png", alt="rush", width=800, class="center" )}}

# Flips

```cpp
enum FlipState
{
    FLIP_READY,
    FLIP_STARTED,
    FLIP_RECOVER,
    FLIP_RETURN
};
```

```cpp
if (flip_state == FLIP_READY)
{
    dist_pid.output_value = 100.0f;
    float angular = orient_pid.compute(orient_sensor, dt, true);

    left_motor_pct = constrain((100.0f - angular) * MOTOR_SCALE, -100.0f, 100.0f);
    right_motor_pct = constrain((100.0f + angular) * MOTOR_SCALE, -100.0f, 100.0f);
    setMotors(left_motor_pct, right_motor_pct);

    if (dist_estimate <= dist_pid.setpoint)
    {
        flip_state = FLIP_STARTED;
        flip_time = current_control_time;
    }
}
```

```cpp
else if (flip_state == FLIP_STARTED)
{
    dist_pid.output_value = -100.0f;
    left_motor_pct = -100.0f * MOTOR_SCALE;
    right_motor_pct = -100.0f * MOTOR_SCALE;
    setMotors(left_motor_pct, right_motor_pct);
    if (abs(dmp_roll) > 50.0f)
    {
        flip_state = FLIP_RECOVER;
        flip_time = current_control_time;
    }
}
```

```cpp
else if (flip_state == FLIP_RECOVER)
{

    dist_pid.output_value = 0.0f;
    left_motor_pct = 0.0f;
    right_motor_pct = 0.0f;
    setMotors(left_motor_pct, right_motor_pct);

    if (abs(dmp_roll) > 175.0f)
    {
        flip_state = FLIP_RETURN;
        yaw_offset = dmp_yaw;
        orient_pid.reset();
        float new_start_dist = (tof2_dist > 10.0f) ? tof2_dist : 3000.0f;
        dist_pid.resetKalman(new_start_dist);
    }
}
```


```cpp
else if (flip_state == FLIP_RETURN)
{
    dist_pid.output_value = -100.0f;
    float angular = -orient_pid.compute(orient_sensor, dt, true);
    left_motor_pct = constrain(-(100.0f - angular) * MOTOR_SCALE, -100.0f, 100.0f);
    right_motor_pct = constrain(-(100.0f + angular) * MOTOR_SCALE, -100.0f, 100.0f);
    setMotors(left_motor_pct, right_motor_pct);
}
```

## Trial 1
[Video Here](https://youtube.com/shorts/0lrJJ1Pl7HM)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/0lrJJ1Pl7HM"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab8/flip1.png", alt="flip1", width=800, class="center" )}}

## Trial 2
[Video Here](https://youtube.com/shorts/yrgW7axrOVk)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/yrgW7axrOVk"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab8/flip2.png", alt="flip2", width=800, class="center" )}}


## Trial 3

[Video Here](https://youtube.com/shorts/wNWp9pbFn4k)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/wNWp9pbFn4k"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab8/flip3.png", alt="flip3", width=800, class="center" )}}

## Velocity Analysis

## Blooper


[Video Here](https://youtube.com/shorts/gRz22fiRBk0)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/gRz22fiRBk0"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

