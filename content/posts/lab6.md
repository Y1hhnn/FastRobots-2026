+++
title = "Lab 6: Orientation Control"
date = "2026-03-18"
+++

The objective of this lab is to design, implement, and evaluate an orientation controller for the robot using the IMU, specifically by estimating yaw from gyroscope data and applying a P/PI/PD/PID-style controller to command differential motor motion for in-place rotation.

# Pre-lab

The overall software architecture and Bluetooth communication framework in this lab follow the same structure developed in Lab 5. The robot uses a bidirectional BLE interface to receive commands from a host computer and transmit logged sensor and control data back for analysis.

Compared to Lab 5, I introduce an additional abstraction layer through `ControlMode`, allowing the robot to switch between different feedback strategies. Specifically, a new command `SET_MODE` is implemented to toggle between:

- **MODE_POSITION**: uses ToF distance measurements as feedback.
- **MODE_ORIENTATION**: uses IMU-derived yaw (from gyroscope integration) as feedback.

# PID Input Signal

A gyroscope measures angle velocity. Integration is needed to estimate orientation. This can cause bias and drift, as errors can accumulate. 

```cpp
unsigned long current_imu_time = micros();
float dt = (current_imu_time - last_imu_time) / 1.e6;
gyr_z = myICM.gyrZ();
gyr_yaw += (gyr_z - gyr_bias_z) * dt;
```

```cpp
void setupIMU()
{
    //...
    // IMU initialization
    //...

    SERIAL_PORT.println("Initializing DMP...");
    bool success = true;
    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    // Enable the DMP orientation sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    // Set DMP ODR to to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    // Check success
    if (success)
        SERIAL_PORT.println(F("DMP enabled!"));
    else
        SERIAL_PORT.println("DMP initialization failed!");

}
```

```cpp
bool updateIMU()
{
    if (myICM.dataReady())
    {
        myICM.getAGMT();
        acc_x = myICM.accX();
        gyr_z = myICM.gyrZ() - gyr_z_offset;
    }

    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        if ((data.header & DMP_header_bitmap_Quat6) > 0)
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // X
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Y
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Z

            double q0_sq = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
            if (q0_sq < 0.0)
                q0_sq = 0.0;
            double q0 = sqrt(q0_sq); // W
            dmp_yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * 180.0 / PI;

            imu_count++;

            return true;
        }
    }
    return false;
}
```

# PID Algorithm

To make the robot turn in place, I need to send speend commands of equal magnitude but opposite directions to the left and right wheels. The output of the PID controller is the signal used to control the wheel speeds.

```cpp
void runController()
{

    case MODE_ORIENTATION:
    {
        float new_sensor = wrapAngle180(getSensorValue(dt) - yaw_offset);
        float new_error = wrapAngle180(new_sensor - setpoint);

        // Avoid derivative Kick
        float delta_angle = wrapAngle180(new_sensor - sensor_value);
        raw_derivative_value = delta_angle / dt;
        // Derivative LPF
        derivative_value = derivative_filter_alpha * raw_derivative_value + (1.0f - derivative_filter_alpha) * derivative_value;
        // Anti-Windup
        raw_integral_value = integral_value + new_error * dt;
        float new_integral = constrain(raw_integral_value, -integral_limit, integral_limit);
        
        float unsat_output = kp * new_error + ki * new_integral + kd * derivative_value;
        bool saturated_high = unsat_output > output_limit;
        bool saturated_low = unsat_output < -output_limit;
        if ((!saturated_high && !saturated_low) || (saturated_high && new_error < 0) || (saturated_low && new_error > 0))
        {
            integral_value = new_integral;
        }
        output_value = kp * new_error + ki * integral_value + kd * derivative_value;
        error_value = new_error;
        sensor_value = new_sensor;
        applyOutput(output_value);
        break;
    }
}
```

```cpp
void applyOutput(float output)
{
    if (control_mode == MODE_ORIENTATION)
    {
        left_motor_pct = power;
        right_motor_pct = -power;
        setMotors(power, -power);
    }
}
```

## Derivative Term

```cpp
// Avoid derivative Kick
float delta_angle = wrapAngle180(new_sensor - sensor_value);
raw_derivative_value = delta_angle / dt;
```

```cpp
// Derivative LPF
derivative_value = derivative_filter_alpha * raw_derivative_value + (1.0f - derivative_filter_alpha) * derivative_value;
```

## Integrate Term

```cpp
// Anti-Windup
raw_integral_value = integral_value + new_error * dt;
float new_integral = constrain(raw_integral_value, -integral_limit, integral_limit);
```

# P-Control

[Video Here](https://youtube.com/shorts/1Asy4QV-4r8)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/1Asy4QV-4r8"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab6/2.5_0_0.png", alt="P-Control", width=700, class="center" )}}


# PD-Control
[Video Here](https://youtube.com/shorts/j0WzvpoUmRU)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/j0WzvpoUmRU"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab6/2.5_0_0.25.png", alt="PD-Control", width=700, class="center" )}}

# PID-Control
[Video Here](https://youtube.com/shorts/mU_Simorh5g)
<div style="width:100%;height:0;position:relative;padding-bottom:64.923%;">
  <iframe
    src="https://youtube.com/embed/mU_Simorh5g"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width:100%;height:100%;position:absolute;left:0;top:0;overflow:hidden;">
  </iframe>
</div>

{{ image(path="content/posts/lab6/2.5_1.2_0.25.png", alt="PD-Control", width=700, class="center" )}}
