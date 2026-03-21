+++
title = "Lab 5: Linear PID Control and Linear Interpolation"
date = "2026-03-11"
+++

This lab aims to design and validate a fast, reliable closed-loop position controller for the robot using ToF distance feedback, with the specific goal of driving toward a wall and stopping at a desired stand-off distance.

# Pre-lab

## Code Structure

I reorganized my Arduino program into several modular sections so that each could be developed and debugged independently. 
 
Every single loop now handles Bluetooth communication, updates sensor readings, runs the controller when the robot is active, and stores sampled data for later transmission. 

 ```cpp
 void loop()
{
    handleBLE();
    updateSensors();
    if (active && sensor_updated)
    {
        runController();
        pid_count++;
        sensor_updated = false;
    }
    if (collecting)
        collectSamples();
    if (millis() - start_sample_time >= SAMPLE_DURATION)
        stopRobot();
}
 ```

## BLE Handler

To support flexible interaction between the computer and the robot, I implemented new commands as followoing:
- **PING:** Used to verify communication. The robot responds with “PONG”.
- **START_RECORD**: Initializes the system, clears previous logs, takes an initial sensor reading, and starts the control loop and data collection. This command triggers the main experiment run.
- **STOP_ROBOT**:Immediately stops the motors and ends the current run.
- **SEND_LOG**: Sends all recorded data (time, motor outputs, PID terms, IMU, and ToF readings) back to the computer for plotting and analysis.
- **UPDATE_PID (kp, ki, kd)**: Updates the proportional, integral, and derivative gains of the controller in real time, allowing rapid tuning.
- **SET_DURATION (ms)**: set the duration of control run for experiment.
- **SET_SETPOINT (value)**: Sets the target value for the controller. For this lab, it corresponds to the desired distance from the wall.
- **SET_MOTOR_MAX(0-100)**: Limits the maximum motor output percentage to prevent overly aggressive motion.
- **SET_EXTRAPOLATION**: Enables or disables linear extrapolation of ToF measurements between sensor updates.

In each loop iteration, read_data() checks whether a new string has been written, and if so, `handleCommand()` parses the message using the RobotCommand helper class.

To start a customized control run, the host side sends the following commands:
```python
ble.send_command(CMD.SET_DURATION, "8000")
ble.send_command(CMD.SET_SETPOINT, "304")
ble.send_command(CMD.SET_MOTOR_MAX, "10")
ble.send_command(CMD.UPDATE_PID, "0.2|0|0")
ble.send_command(CMD.SET_EXTRAPOLATION, "0")

ble.send_command(CMD.START_RECORD, "")
```

During experiment run,  Artemis stores sampled values in arrays. When the robot stops, the computer can send the `SEND_LOG` command, which iterates through the buffers, formats, and sends to the Python side.

{{ image(path="content/posts/lab5/send_log.png", alt="send_log", width=1000, class="center" )}}


For safety and robustness, I also implemented a hard stop directly on the Artemis. If the Bluetooth connection is lost while the robot is active, the code detects the disconnection in `handleBLE()` and immediately calls `stopRobot()` to shut off the motors. In addition, the control run automatically terminates after the specified sample duration.

# Position Control

The position controller was implemented in runController(). At each control step, the code computes the current distance error from the ToF sensor, updates the integral and derivative terms, and then forms the control output using the PID equation. The resulting output is then saturated and passed to `applyOutput()`

```python
void runController()
{
    unsigned long current_control_time = micros();
    float dt = (current_control_time - last_control_time) / 1.e6;
    if (dt <= 0.0f) return;
    last_control_time = current_control_time;

    sensor_value = getSensorValue();
    float new_error = setpoint - sensor_value;

    integral_value += new_error * dt;
    derivative_value = (new_error - error_value) / dt;

    output_value = kp * new_error + ki * integral_value + kd * derivative_value;
    error_value = new_error;

    applyOutput(output_value);
}

void applyOutput(float output)
{
    float power = -output;
    if (power > MAX_MOTOR_PCT)
        power = MAX_MOTOR_PCT;
    if (power < -MAX_MOTOR_PCT);

    power = -MAX_MOTOR_PCT;
    left_motor_pct = power;
    right_motor_pct = power;
    setMotors(power, power);
}
```


Before tuning the full PID controller, I estimated a reasonable range for the proportional gain.

 In my implementation, the controller computes the error as the difference between the desired stand-off distance and the measured ToF distance, then converts that error into a motor command through the PID equation. 
 
 If the robot starts about `2–4 m` from the wall and the target setpoint is `304 mm`, then the largest likely error is roughly `1700–3700 mm`. So,  `K_p` should be on the order of `100/4000 = 0.025` to `100/2000 = 0.05 ` motor-percent per millimeter. 

A gain on the order of  `10^{-2}` to `10^{-1}\ \%/\text{mm}` is reasonable. In particular, values around 0.03–0.06\ \%/\text{mm} allow large distance errors to produce strong motion while still leaving enough dynamic range for the controller to slow the robot as it approaches the setpoint.

## P-Control Test
I first set `K_p = 0.025`, `K_i = 0`, and `K_d = 0`, so the controller reduces to `u = K_p e`

{{ image(path="content/posts/lab5/p_10.png", alt="p_10", width=1000, class="center" )}}
