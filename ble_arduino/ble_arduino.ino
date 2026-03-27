#include <SparkFun_VL53L1X.h>
#include <ICM_20948.h>
#include <Wire.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "math.h"
#include <BasicLinearAlgebra.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "d427e7cc-c400-4597-b417-d564e20d6600"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

BLEDevice central;
//////////// BLE UUIDs ////////////

//////////// ICM Sensor ////////////
#define SERIAL_PORT Serial
#define SPI_PORT SPI   // Your desired SPI port.
#define CS_PIN 2       // Which pin you connect CS to.
#define WIRE_PORT Wire // Your desired Wire port.

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;
//////////// ICM Sensor ////////////

//////////// TOF Sensor ////////////
#define XSHUT_PIN A2 //// XSHUT pin for the second sensor
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
//////////// TOF Sensor ////////////

//////////// Motors ////////////
#define LEFT_MOTOR_IN1 11
#define LEFT_MOTOR_IN2 12
#define RIGHT_MOTOR_IN1 13
#define RIGHT_MOTOR_IN2 14

float MOTOR_SCALE = 1;

// --- Calibration Constants ---
const int FWD_LEFT_MIN = 29;
const int FWD_LEFT_MED = 99;
const int FWD_LEFT_MAX = 255;
const int FWD_RIGHT_MIN = 42;
const int FWD_RIGHT_MED = 148;
const int FWD_RIGHT_MAX = 255;
const int BWD_LEFT_MIN = 34;
const int BWD_LEFT_MED = 112;
const int BWD_LEFT_MAX = 255;
const int BWD_RIGHT_MIN = 46;
const int BWD_RIGHT_MED = 150;
const int BWD_RIGHT_MAX = 255;
//////////// Motors ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// IMU
unsigned long last_imu_time = 0; // in microseconds
// float last_acc_roll, last_acc_pitch;
// float last_gyr_roll, last_gyr_pitch;
// float last_comp_roll, last_comp_pitch;
// float comp_roll, comp_pitch;
float acc_x, acc_y;
float gyr_z, gyr_yaw, gyr_bias_z;
int imu_count = 0;
bool imu_init = true;
// float filt_alpha = 0.15;
// float comp_alpha = 0.9;

// TOF
float tof1_dist, tof2_dist;
float tof2_velocity;
unsigned long tof2_time;
int tof_count = 0;

// Motor
float left_motor_pct = 0.0f;
float right_motor_pct = 0.0f;

// Controller
bool active = false;
bool sensor_updated = false;
float kp = 0.05f;
float ki = 0.0f;
float kd = 0.0f;
int pid_count = 0;

unsigned long last_control_time = 0; // in microseconds
float setpoint = 0.0f;               // Degree for IMU, cm for TOF
float sensor_value = 0.0f;
float error_value = 0.0f;
float integral_value = 0.0f;
float derivative_value = 0.0f;
float raw_derivative_value = 0.0f;
float raw_integral_value = 0.0f;
float output_value = 0.0f;

float derivative_filter_alpha = 0.2f;
float integral_limit = 100.0f;
float output_limit = 100.0f;
//////////// Global Variables ////////////

//////////// Sample Data ////////////
const int SAMPLE_LEN = 1500;
int SAMPLE_INTERVAL = 1000;          // in microseconds
unsigned long last_sample_time = 0;  // in microseconds
int SAMPLE_DURATION = 5000;          // in milliseconds
unsigned long start_sample_time = 0; // in milliseconds
int sample_count = 0;
bool collecting = false;

// System Buffers
unsigned long time_buffer[SAMPLE_LEN];

// Sensor Buffers
float sensor_buffer[SAMPLE_LEN];
float tof_1_buffer[SAMPLE_LEN];
float tof_2_buffer[SAMPLE_LEN];
float acc_x_buffer[SAMPLE_LEN];
float acc_y_buffer[SAMPLE_LEN];
float gyr_z_buffer[SAMPLE_LEN];
float yaw_buffer[SAMPLE_LEN];

// Motor Buffer
float left_pwm[SAMPLE_LEN];
float right_pwm[SAMPLE_LEN];
float left_percent[SAMPLE_LEN];
float right_percent[SAMPLE_LEN];

// PID Buffer
float error_buffer[SAMPLE_LEN];
float derivative_buffer[SAMPLE_LEN];
float raw_derivative_buffer[SAMPLE_LEN];
float raw_integral_buffer[SAMPLE_LEN];
float integral_buffer[SAMPLE_LEN];
//////////// Sample Data ////////////

//////////// Commands ////////////
enum CommandTypes
{
    PING,
    START_RECORD,
    STOP_ROBOT,
    SEND_LOG,
    UPDATE_PID,
    SET_DURATION,
    SET_SETPOINT,
    SET_MODE,
    SET_MOTOR_SCALE,
    SET_EXTRAPOLATION,
    SET_KALMANFILTER,
    SET_FLIP_DURATION
};
//////////// Commands ////////////

//////////// Control Mode ////////////
enum ControlMode
{
    MODE_POSITION,
    MODE_ORIENTATION,
    MODE_RUSH,
    MODE_IDLE,
    MODE_FLIP
};

enum FlipState
{
    FLIP_READY,
    FLIP_STARTED,
    FLIP_COMPLETED,
    FLIP_IDLE
};

ControlMode control_mode = MODE_POSITION;
FlipState flip_state = FLIP_READY;
int flip_duration = 500000;  // in microseconds
unsigned long flip_time = 0; // in microseconds
bool extrapolation = true;
bool kalman_filter = false;
//////////// Control Mode ////////////

//////////// Kalman Filter ////////////
using namespace BLA;
float sys_d = 0.000309f;
float sys_m = 10.579281f;

Matrix<2, 2> A = {0.0f, 1.0f, 0.0f, -sys_d / sys_m};
Matrix<2, 1> B = {0.0f, 1.0f / sys_m};

Matrix<2, 1> mu = {0.0f, 0.0f};
Matrix<2, 2> Sigma = {10000.0f, 0.0f, 0.0f, 10000.0f};

Matrix<1, 2> C = {1.0f, 0.0f};
// Process Noise
Matrix<2, 2> Sigma_u = {97.0f * 97.0f, 0.0f, 0.0f, 97.0f * 97.0f};
// Measurement Noise
Matrix<1, 1> Sigma_z = {15.0f * 15.0f};

Matrix<2, 2> I2 = {1.0f, 0.0f, 0.0f, 1.0f};
//////////// Kalman Filter ////////////

// =========================
// SETUP
// =========================
void setup()
{
    Serial.begin(115200);
    setupBle();
    setupICM();
    setupToF();
    setupMotors();
    led_blink(3, 200);
}

// =========================
// Main Loop
// =========================
void loop()
{
    handleBLE();
    updateSensors();
    if (active && (sensor_updated || extrapolation || kalman_filter))
    {
        runController();
        pid_count++;
        sensor_updated = false;
    }
    if (collecting)
        collectSamples();
    if (active && millis() - start_sample_time >= SAMPLE_DURATION)
        stopRobot();
}

// =========================
// BLE
// =========================
void handleBLE()
{
    if (!central)
    {
        central = BLE.central();
        if (central)
        {
            Serial.print("Connected to: ");
            Serial.println(central.address());
        }
    }

    if (central && central.connected())
    {
        read_data();
    }
    else if (central && !central.connected())
    {
        Serial.println("Disconnected from: ");
        Serial.print(central.address());
        if (active)
        {
            Serial.println("Failsafe: Connection lost, stopping robot!");
            stopRobot();
        }
        central = BLEDevice();
    }
}

void handleCommand()
{
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    success = robot_cmd.get_command_type(cmd_type);
    // Check if the last tokenization was successful and return if failed
    if (!success)
        return;

    // Handle the command type accordingly
    switch (cmd_type)
    {
    case PING:
    {
        tx_estring_value.clear();
        tx_estring_value.append("PONG");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        break;
    }

    case START_RECORD:
    {
        cleanLog();
        cleanState();

        if (control_mode == MODE_POSITION || control_mode == MODE_FLIP ||
            control_mode == MODE_RUSH || control_mode == MODE_IDLE)
        {
            distanceSensor2.stopRanging();
            // distanceSensor1.startRanging();
            distanceSensor2.startRanging();
            Serial.println("Waiting for second ToF reading...");
            while (!distanceSensor2.checkForDataReady())
            {
                delay(1);
            }
            tof2_dist = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            if (control_mode == MODE_FLIP)
            {
                flip_state = (tof2_dist < setpoint) ? FLIP_IDLE : FLIP_READY;
            }
            sensor_updated = true;
            sensor_value = tof2_dist;
            tof2_time = micros();
            tof2_velocity = 0.0f;

            // reset Kalman Filter state
            mu(0, 0) = tof2_dist;
            mu(1, 0) = 0.0f;
            Sigma = {10000.0f, 0.0f, 0.0f, 10000.0f};
        }
        if (control_mode == MODE_ORIENTATION || control_mode == MODE_IDLE)
        {
            imu_init = true;
            while (!myICM.dataReady())
            {
                delay(1);
            }
            updateIMU();
            sensor_updated = true;
            sensor_value = gyr_yaw;
        }

        sample_count = 0;

        last_sample_time = micros();
        last_control_time = micros();
        start_sample_time = millis();

        collecting = true;
        active = true;
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Started Recording");
        break;
    }

    case STOP_ROBOT:
    {
        stopRobot();
        break;
    }

    case SEND_LOG:
    {
        for (int i = 0; i < sample_count; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append((int)time_buffer[i]);
            tx_estring_value.append("|LPWM: ");
            tx_estring_value.append(left_pwm[i]);
            tx_estring_value.append("|RPWM: ");
            tx_estring_value.append(right_pwm[i]);
            tx_estring_value.append("|LPEC: ");
            tx_estring_value.append(left_percent[i]);
            tx_estring_value.append("|RPEC: ");
            tx_estring_value.append(right_percent[i]);
            // tx_estring_value.append("|E: ");
            // tx_estring_value.append(error_buffer[i]);
            // tx_estring_value.append("|I: ");
            // tx_estring_value.append(integral_buffer[i]);
            // tx_estring_value.append("|RI: ");
            // tx_estring_value.append(raw_integral_buffer[i]);
            // tx_estring_value.append("|D: ");
            // tx_estring_value.append(derivative_buffer[i]);
            // tx_estring_value.append("|RD: ");
            // tx_estring_value.append(raw_derivative_buffer[i]);
            tx_estring_value.append("|AX: ");
            tx_estring_value.append(acc_x_buffer[i]);
            // tx_estring_value.append("|GZ: ");
            // tx_estring_value.append(gyr_z_buffer[i]);
            // tx_estring_value.append("|YW: ");
            // tx_estring_value.append(yaw_buffer[i]);
            // tx_estring_value.append("|T1: ");
            // tx_estring_value.append(tof_1_buffer[i]);
            tx_estring_value.append("|T2: ");
            tx_estring_value.append(tof_2_buffer[i]);
            tx_estring_value.append("|S: ");
            tx_estring_value.append(sensor_buffer[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(3);
        }
        tx_estring_value.clear();
        tx_estring_value.append("Sample Count: ");
        tx_estring_value.append(sample_count);
        tx_estring_value.append("| ToF Count: ");
        tx_estring_value.append(tof_count);
        tx_estring_value.append("| IMU Count: ");
        tx_estring_value.append(imu_count);
        tx_estring_value.append("| PID Count: ");
        tx_estring_value.append(pid_count);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        break;
    }

    case UPDATE_PID:
    {
        float new_kp, new_ki, new_kd;
        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(new_kp);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_ki);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_kd);
        if (!success)
            return;

        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        break;
    }

    case SET_DURATION:
    {
        int new_duration;
        success = robot_cmd.get_next_value(new_duration);
        if (!success)
            return;
        SAMPLE_DURATION = new_duration;
        Serial.print("Set Sample Duration to: ");
        Serial.println(SAMPLE_DURATION);
        break;
    }

    case SET_SETPOINT:
    {
        float new_setpoint;
        success = robot_cmd.get_next_value(new_setpoint);
        if (!success)
            return;
        setpoint = new_setpoint;
        Serial.println("Set Setpoint to: ");
        Serial.print(setpoint);
        break;
    }

    case SET_MODE:
    {
        int new_mode;
        success = robot_cmd.get_next_value(new_mode);
        if (!success)
            return;
        switch (ControlMode(new_mode))
        {
        case MODE_POSITION:
            control_mode = MODE_POSITION;
            break;
        case MODE_ORIENTATION:
            control_mode = MODE_ORIENTATION;
            break;
        case MODE_RUSH:
            control_mode = MODE_RUSH;
            break;
        case MODE_FLIP:
            control_mode = MODE_FLIP;
            break;
        case MODE_IDLE:
            control_mode = MODE_IDLE;
            break;

        default:
            Serial.print("Invalid Control Mode: ");
            Serial.println(new_mode);
        }
        break;
    }

    case SET_MOTOR_SCALE:
    {
        float new_scale;
        success = robot_cmd.get_next_value(new_scale);
        if (!success)
            return;
        MOTOR_SCALE = new_scale;
        Serial.print("Set Motor Scale to: ");
        Serial.println(MOTOR_SCALE);
        break;
    }

    case SET_EXTRAPOLATION:
    {
        int extrapolation_int;
        success = robot_cmd.get_next_value(extrapolation_int);
        if (!success)
            return;
        extrapolation = (extrapolation_int != 0);
        kalman_filter = (extrapolation_int == 0);
        Serial.print("Set Extrapolation to: ");
        Serial.println(extrapolation ? "True" : "False");
        break;
    }

    case SET_KALMANFILTER:
    {
        int kalman_filter_int;
        success = robot_cmd.get_next_value(kalman_filter_int);
        if (!success)
            return;
        kalman_filter = (kalman_filter_int != 0);
        extrapolation = (kalman_filter_int == 0);
        Serial.print("Set Kalman Filter to: ");
        Serial.println(kalman_filter ? "True" : "False");
        break;
    }

    case SET_FLIP_DURATION:
    {
        int new_flip_duration;
        success = robot_cmd.get_next_value(new_flip_duration);
        if (!success)
            return;
        flip_duration = new_flip_duration;
        Serial.print("Set Flip Duration to: ");
        Serial.print(flip_duration);
        Serial.println(" microseconds");
        break;
    }

    default:
    {
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
    }
    }
}

// =========================
// Controller
// =========================
void runController()
{
    unsigned long current_control_time = micros();
    float dt = (current_control_time - last_control_time) / 1.e6; // in seconds
    if (dt <= 0.0f)
        return;
    last_control_time = current_control_time;

    switch (control_mode)
    {
    case MODE_POSITION:
    {
        float new_sensor = getSensorValue(dt);
        float new_error = new_sensor - setpoint;

        if (kalman_filter)
        {
            derivative_value = mu(1, 0);
        }
        else
        {
            // Avoid derivative Kick
            raw_derivative_value = (new_error - error_value) / dt;
            // Derivative LPF
            derivative_value = derivative_filter_alpha * raw_derivative_value + (1.0f - derivative_filter_alpha) * derivative_value;
        }

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
        break;
    }

    case MODE_ORIENTATION:
    {
        float new_sensor = getSensorValue(dt);
        float new_error = new_sensor - setpoint;
        break;
    }

    case MODE_RUSH:
    {
        float new_sensor = getSensorValue(dt);
        sensor_value = new_sensor;
        float new_error = new_sensor - setpoint;
        // if (new_error > 0)
        output_value = 100.0f;
        // else
        //     output_value = 0.0f;

        break;
    }

    case MODE_IDLE:
    {
        output_value = 0.0f;
        break;
    }

    case MODE_FLIP:
    {
        float new_sensor = getSensorValue(dt);
        float new_error = setpoint - new_sensor;

        sensor_value = new_sensor;
        error_value = new_error;
        raw_derivative_value = 0.0f;
        derivative_value = 0.0f;
        raw_integral_value = 0.0f;
        integral_value = 0.0f;

        if (flip_state == FLIP_READY)
        {
            if (new_error > 0)
            {
                flip_state = FLIP_STARTED;
                flip_time = current_control_time;
                output_value = -100;
                break;
            }
        }
        if (flip_state == FLIP_STARTED)
        {
            if (current_control_time - flip_time > flip_duration)
            {
                flip_state = FLIP_COMPLETED;
                output_value = 100;
                break;
            }
            output_value = -100;
        }
        if (flip_state == FLIP_COMPLETED)
        {
            output_value = 100;
        }
        break;
    }

    default:
    {
        break;
    }
    }

    applyOutput(output_value);
}

float getSensorValue(float dt)
{
    if (control_mode == MODE_POSITION || control_mode == MODE_FLIP || control_mode == MODE_RUSH)
    {
        if (extrapolation)
        {
            unsigned long current_time = micros();
            float extrapolated_dist = tof2_dist + tof2_velocity * ((current_time - tof2_time) / 1.e6);
            return extrapolated_dist;
        }
        else if (kalman_filter)
        {
            Matrix<2, 2> Ad = I2 + A * dt;
            Matrix<2, 1> Bd = B * dt;
            float u_t = output_value / 100.0f;
            Matrix<1, 1> u_vec = {u_t};
            Matrix<2, 1> mu_p = Ad * mu + Bd * u_vec;
            Matrix<2, 2> Sigma_p = Ad * Sigma * (~Ad) + Sigma_u;
            if (!sensor_updated)
            {
                mu = mu_p;
                Sigma = Sigma_p;
                return mu_p(0, 0);
            }
            Matrix<1, 1> y = {tof2_dist};
            Matrix<1, 1> y_m = y - C * mu_p;
            Matrix<1, 1> S = C * Sigma_p * (~C) + Sigma_z;
            Matrix<1, 1> S_inv;
            S_inv(0, 0) = 1.0f / S(0, 0);
            Matrix<2, 1> K = Sigma_p * (~C) * S_inv;
            mu = mu_p + K * y_m;
            Sigma = (I2 - K * C) * Sigma_p;
            return mu(0, 0);
        }
        return tof2_dist;
    }
    else if (control_mode == MODE_ORIENTATION)
    {
        return gyr_yaw;
    }
    return 0.0f;
}

void applyOutput(float output)
{
    float power = output * MOTOR_SCALE;
    if (power > output_limit)
        power = output_limit;
    if (power < -output_limit)
        power = -output_limit;

    if (control_mode == MODE_POSITION || control_mode == MODE_FLIP ||
        control_mode == MODE_RUSH || control_mode == MODE_IDLE)
    {
        left_motor_pct = power;
        right_motor_pct = power;
        setMotors(power, power);
    }
    else if (control_mode == MODE_ORIENTATION)
    {
        left_motor_pct = power;
        right_motor_pct = -power;
        setMotors(power, -power);
    }
    else
    {
        left_motor_pct = 0;
        right_motor_pct = 0;
        setMotors(0, 0);
    }
}

// =========================
// Sensors
// =========================
void updateSensors()
{
    if (myICM.dataReady())
    {
        updateIMU();
        if (control_mode == MODE_ORIENTATION)
            sensor_updated = true;
    }
    // if (distanceSensor1.checkForDataReady()) {
    //     tof1_dist = distanceSensor1.getDistance();
    //     distanceSensor1.clearInterrupt();
    // }

    if (distanceSensor2.checkForDataReady())
    {
        if (extrapolation)
        {
            unsigned long current_time = micros();
            float new_dist = distanceSensor2.getDistance();
            float dt = (current_time - tof2_time) / 1.e6;
            if (dt > 0)
                tof2_velocity = (new_dist - tof2_dist) / dt;

            tof2_time = current_time;
            tof2_dist = new_dist;
            distanceSensor2.clearInterrupt();
        }
        else
        {
            tof2_dist = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
        }
        tof_count++;
        if (control_mode == MODE_POSITION || control_mode == MODE_FLIP ||
            control_mode == MODE_RUSH || control_mode == MODE_IDLE)
            sensor_updated = true;
    }
}

void updateIMU()
{
    unsigned long current_imu_time = micros();
    myICM.getAGMT();
    if (imu_init)
    {
        acc_x = myICM.accX();
        acc_y = myICM.accY();
        gyr_z = myICM.gyrZ();
        gyr_yaw = 0.0f;
        imu_init = false;
    }
    else
    {
        acc_x = myICM.accX();
        acc_y = myICM.accY();
        gyr_z = myICM.gyrZ();
        float dt = (current_imu_time - last_imu_time) / 1.e6;
        gyr_yaw += (gyr_z - gyr_bias_z) * dt;
    }
    last_imu_time = current_imu_time;
    imu_count++;

    // acc_roll = atan2(myICM.accY(), sqrt(myICM.accX()*myICM.accX() + myICM.accZ()*myICM.accZ())) * 180 / M_PI;
    // acc_pitch = atan2(myICM.accX(), sqrt(myICM.accY()*myICM.accY() + myICM.accZ()*myICM.accZ()))* 180 / M_PI;
    // if(imu_init){
    //     gyr_roll =0.0f;
    //     gyr_pitch = 0.0f;
    //     comp_roll = acc_roll;
    //     comp_pitch = acc_pitch;
    //     imu_init = false;
    // }
    // else
    // {
    //     float dt = (current_imu_time - last_imu_time)/1.e6;
    //     acc_roll = filt_alpha * acc_roll + (1 - filt_alpha) * last_acc_roll;
    //     acc_pitch = filt_alpha * acc_pitch + (1 - filt_alpha) * last_acc_pitch;
    //     gyr_roll = last_gyr_roll + myICM.gyrX()*dt;
    //     gyr_pitch = last_gyr_pitch - myICM.gyrY()*dt;
    //     comp_roll = (1-comp_alpha) * acc_roll + comp_alpha * (last_comp_roll + myICM.gyrX()*dt);
    //     comp_pitch = (1-comp_alpha) * acc_pitch + comp_alpha * (last_comp_pitch - myICM.gyrY()*dt);
    // }
    // last_acc_roll = acc_roll;
    // last_acc_pitch = acc_pitch;
    // last_gyr_roll = gyr_roll;
    // last_gyr_pitch = gyr_pitch;
    // last_comp_roll = comp_roll;
    // last_comp_pitch = comp_pitch;
}

// =========================
// Motors
// =========================
void setMotors(float left_percent, float right_percent)
{
    left_percent = constrain(left_percent, -100, 100);
    right_percent = constrain(right_percent, -100, 100);

    int left_pwm = percentToPWM(left_percent, true);
    int right_pwm = percentToPWM(right_percent, false);

    // left motor
    if (left_percent > 0)
    {
        analogWrite(LEFT_MOTOR_IN1, left_pwm);
        analogWrite(LEFT_MOTOR_IN2, 0);
    }
    else if (left_percent < 0)
    {
        analogWrite(LEFT_MOTOR_IN1, 0);
        analogWrite(LEFT_MOTOR_IN2, left_pwm);
    }
    else
    {
        analogWrite(LEFT_MOTOR_IN1, 0);
        analogWrite(LEFT_MOTOR_IN2, 0);
    }

    // right motor
    if (right_percent > 0)
    {
        analogWrite(RIGHT_MOTOR_IN1, right_pwm);
        analogWrite(RIGHT_MOTOR_IN2, 0);
    }
    else if (right_percent < 0)
    {
        analogWrite(RIGHT_MOTOR_IN1, 0);
        analogWrite(RIGHT_MOTOR_IN2, right_pwm);
    }
    else
    {
        analogWrite(RIGHT_MOTOR_IN1, 0);
        analogWrite(RIGHT_MOTOR_IN2, 0);
    }
}

int percentToPWM(float percent, bool isLeft)
{
    percent = constrain(percent, -100, 100);
    if (percent == 0.0)
        return 0;

    bool forward = (percent > 0);
    float p = fabsf(percent) / 100.0f;

    if (isLeft)
    {
        if (forward)
        {
            if (p <= 0.5)
            {
                float t = p / 0.5;
                return FWD_LEFT_MIN + t * (FWD_LEFT_MED - FWD_LEFT_MIN);
            }
            else
            {
                float t = (p - 0.5) / 0.5;
                return FWD_LEFT_MED + t * (FWD_LEFT_MAX - FWD_LEFT_MED);
            }
        }
        else
        {
            if (p <= 0.5)
            {
                float t = p / 0.5;
                return BWD_LEFT_MIN + t * (BWD_LEFT_MED - BWD_LEFT_MIN);
            }
            else
            {
                float t = (p - 0.5) / 0.5;
                return BWD_LEFT_MED + t * (BWD_LEFT_MAX - BWD_LEFT_MED);
            }
        }
    }
    else
    {
        if (forward)
        {
            if (p <= 0.5)
            {
                float t = p / 0.5;
                return FWD_RIGHT_MIN + t * (FWD_RIGHT_MED - FWD_RIGHT_MIN);
            }
            else
            {
                float t = (p - 0.5) / 0.5;
                return FWD_RIGHT_MED + t * (FWD_RIGHT_MAX - FWD_RIGHT_MED);
            }
        }
        else
        {
            if (p <= 0.5)
            {
                float t = p / 0.5;
                return BWD_RIGHT_MIN + t * (BWD_RIGHT_MED - BWD_RIGHT_MIN);
            }
            else
            {
                float t = (p - 0.5) / 0.5;
                return BWD_RIGHT_MED + t * (BWD_RIGHT_MAX - BWD_RIGHT_MED);
            }
        }
    }
}

// =========================
// Logging
// =========================
void collectSamples()
{
    unsigned long current_time = micros();
    if (current_time - last_sample_time < SAMPLE_INTERVAL)
        return;

    if (sample_count >= SAMPLE_LEN)
    {
        collecting = false;
        Serial.println("Sample buffer full, stopping collection");
        digitalWrite(LED_BUILTIN, LOW);
        return;
    }

    last_sample_time = current_time;

    // System Buffers
    time_buffer[sample_count] = millis() - start_sample_time;

    // Sensor Buffers
    sensor_buffer[sample_count] = sensor_value;
    // tof_1_buffer[sample_count] = tof1_dist;
    tof_2_buffer[sample_count] = tof2_dist;
    acc_x_buffer[sample_count] = acc_x;
    // acc_y_buffer[sample_count] = acc_y;
    // gyr_z_buffer[sample_count] = gyr_z;
    // yaw_buffer[sample_count] = gyr_yaw;

    // Motor Buffer
    left_pwm[sample_count] = (left_motor_pct > 0 ? 1 : -1) * percentToPWM(left_motor_pct, true);
    right_pwm[sample_count] = (right_motor_pct > 0 ? 1 : -1) * percentToPWM(right_motor_pct, false);
    left_percent[sample_count] = left_motor_pct;
    right_percent[sample_count] = right_motor_pct;

    // PID Buffer
    error_buffer[sample_count] = error_value;
    derivative_buffer[sample_count] = derivative_value;
    raw_derivative_buffer[sample_count] = raw_derivative_value;
    raw_integral_buffer[sample_count] = raw_integral_value;
    integral_buffer[sample_count] = integral_value;

    sample_count++;
}

void cleanLog()
{
    for (int i = 0; i < SAMPLE_LEN; i++)
    {
        time_buffer[i] = 0;
        left_pwm[i] = 0.0f;
        right_pwm[i] = 0.0f;
        left_percent[i] = 0.0f;
        right_percent[i] = 0.0f;
        sensor_buffer[i] = 0.0f;
        acc_x_buffer[i] = 0.0f;
        // acc_y_buffer[i] = 0.0f;
        // gyr_z_buffer[i] = 0.0f;
        // yaw_buffer[i] = 0.0f;
        // tof_1_buffer[i] = 0.0f;
        tof_2_buffer[i] = 0.0f;
        error_buffer[i] = 0.0f;
        derivative_buffer[i] = 0.0f;
        raw_derivative_buffer[i] = 0.0f;
        integral_buffer[i] = 0.0f;
        raw_integral_buffer[i] = 0.0f;
    }
}

void cleanState()
{
    // IMU
    last_imu_time = 0; // in microseconds
    acc_x = 0.0f;
    acc_y = 0.0f;
    gyr_z = 0.0f;
    gyr_yaw = 0.0f;
    gyr_bias_z = 0.0f;
    imu_init = true;
    imu_count = 0;

    // TOF
    tof1_dist = 0.0f;
    tof2_dist = 0.0f;
    tof_count = 0;
    tof2_velocity = 0.0f;
    tof2_time = 0;

    // Motors
    setMotors(0, 0);
    left_motor_pct = 0.0f;
    right_motor_pct = 0.0f;

    // Controller
    active = false;
    sensor_updated = false;
    last_control_time = 0;
    sensor_value = 0.0f;
    error_value = 0.0f;
    integral_value = 0.0f;
    derivative_value = 0.0f;
    raw_derivative_value = 0.0f;
    raw_integral_value = 0.0f;
    output_value = 0.0f;
    pid_count = 0;

    // Sample
    sample_count = 0;
    collecting = false;
    last_sample_time = 0;
    start_sample_time = 0;
}

// =========================
// Helper Functions
// =========================
void stopRobot()
{
    setMotors(0, 0);
    collecting = false;
    if (control_mode == MODE_POSITION || control_mode == MODE_FLIP || control_mode == MODE_RUSH || control_mode == MODE_IDLE)
    {
        // distanceSensor1.stopRanging();
        distanceSensor2.stopRanging();
    }
    active = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Stopped Robot");
}

void setupBle()
{
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void setupICM()
{
    while (!SERIAL_PORT)
    {
        ; // wait for serial port to connect. Needed for native USB
    }

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(WIRE_PORT, AD0_VAL);

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.println("Trying again...");
            delay(500);
        }
        else
        {
            calibrateGyroBias();
            initialized = true;
        }
    }
}

void setupToF()
{
    // // Turn off Sensor 2 to prevent I2C address conflicts
    // pinMode(XSHUT_PIN, OUTPUT);
    // digitalWrite(XSHUT_PIN, LOW);
    // delay(10);

    // // Initialize Sensor 1
    // while (distanceSensor1.begin(WIRE_PORT) != 0)
    // {
    //     SERIAL_PORT.println("ToF Sensor 1 failed to begin. Retrying in 500ms...");
    //     delay(500);
    // }

    // // Change Sensor 1's I2C address (Default is 0x29, we change it to 0x2A)
    // distanceSensor1.setI2CAddress(0x2A << 1);

    // Turn on Sensor 2
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);

    // Initialize Sensor 2
    while (distanceSensor2.begin(WIRE_PORT) != 0)
    {
        SERIAL_PORT.println("ToF Sensor 2 failed to begin. Retrying in 500ms...");
        delay(500);
    }

    // distanceSensor1.setDistanceModeLong();
    distanceSensor2.setDistanceModeLong();

    SERIAL_PORT.println("Both ToF Sensors online!");
}

void setupMotors()
{
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    analogWriteResolution(8);
    setMotors(0, 0);
    delay(2000);
}

void calibrateGyroBias()
{
    const int N = 500;
    float sum = 0;
    for (int i = 0; i < N; i++)
    {
        while (!myICM.dataReady())
        {
        }
        myICM.getAGMT();
        sum += myICM.gyrZ();
        delay(5);
    }
    gyr_bias_z = sum / N;
}

void led_blink(int times, int delay_time)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delay_time);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delay_time);
    }
}

void write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000)
        {
            tx_float_value = 0;
        }

        previousMillis = currentMillis;
    }
}

void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written())
    {
        handleCommand();
    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}