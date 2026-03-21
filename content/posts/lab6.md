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