#include <SparkFun_VL53L1X.h>
#include <ICM_20948.h>
#include <Wire.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "math.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

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
float acc_x, acc_y;
float gyr_z, dmp_yaw;
// float dmp_pitch = 0.0f;
float dmp_roll = 0.0f;

float gyr_z_offset = 0.0f;
float yaw_offset = 0.0f;
float prev_raw_yaw = 0.0f;
float continuous_yaw = 0.0f;
float continuous_yaw_offset = 0.0f;
int imu_count = 0;

// TOF
float tof1_dist, tof2_dist;
unsigned long tof1_time, tof2_time;
int tof1_count = 0, tof2_count = 0;

// Motor
float left_motor_pct = 0.0f;
float right_motor_pct = 0.0f;
//////////// Global Variables ////////////

//////////// Sample Data ////////////
const int SAMPLE_LEN = 3000;
int SAMPLE_INTERVAL = 1000;          // in microseconds
unsigned long last_sample_time = 0;  // in microseconds
int SAMPLE_DURATION = 5000;          // in milliseconds
unsigned long start_sample_time = 0; // in milliseconds
int sample_count = 0;
bool collecting = false;

// System Buffers
unsigned long time_buffer[SAMPLE_LEN];

// Sensor Buffers
float tof_1_buffer[SAMPLE_LEN];
float tof_2_buffer[SAMPLE_LEN];
float acc_x_buffer[SAMPLE_LEN];
// float acc_y_buffer[SAMPLE_LEN];
float gyr_z_buffer[SAMPLE_LEN];
float yaw_buffer[SAMPLE_LEN];
// float pitch_buffer[SAMPLE_LEN];
float roll_buffer[SAMPLE_LEN];

// Motor Buffer
float left_pwm[SAMPLE_LEN];
float right_pwm[SAMPLE_LEN];
float left_percent[SAMPLE_LEN];
float right_percent[SAMPLE_LEN];

// Distance PID Buffer
float dist_setpoint_buffer[SAMPLE_LEN];
float dist_sensor_buffer[SAMPLE_LEN];
float dist_output_buffer[SAMPLE_LEN];

// Orientation PID Buffer
float orient_setpoint_buffer[SAMPLE_LEN];
float orient_sensor_buffer[SAMPLE_LEN];
float orient_output_buffer[SAMPLE_LEN];
// float orient_error_buffer[SAMPLE_LEN];
// float orient_integral_buffer[SAMPLE_LEN];
// float orient_derivative_buffer[SAMPLE_LEN];
//////////// Sample Data ////////////

////////////////////////// PID Controller /////////////////////////
// =================================================================
// Self-contained PID with optional Kalman filter and extrapolation
// for distance estimation. Orientation mode uses direct sensor input.
// =================================================================
struct PIDController
{
    // --- PID Gains ---
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float setpoint = 0.0f;

    // --- PID State ---
    float sensor_value = 0.0f;
    float error_value = 0.0f;
    float integral_value = 0.0f;
    float derivative_value = 0.0f;
    float raw_derivative_value = 0.0f;
    float raw_integral_value = 0.0f;
    float output_value = 0.0f;

    // --- PID Tuning ---
    float derivative_filter_alpha = 0.2f;
    float integral_limit = 100.0f;
    float output_limit = 100.0f;

    int count = 0;

    // --- Sensor Estimation Mode ---
    enum SensorMode
    {
        DIRECT,
        EXTRAPOLATION,
        KALMAN
    };
    SensorMode sensor_mode = DIRECT;

    // --- Extrapolation State ---
    float extrap_velocity = 0.0f;
    float extrap_last_measurement = 0.0f;
    unsigned long extrap_last_time = 0; // microseconds

    // --- Kalman Filter State ---
    // 2-state model: [position, velocity]
    // x_dot = A*x + B*u,  y = C*x
    Matrix<2, 2> kf_A;
    Matrix<2, 1> kf_B;
    Matrix<1, 2> kf_C;
    Matrix<2, 1> kf_mu;
    Matrix<2, 2> kf_Sigma;
    Matrix<2, 2> kf_Sigma_u; // process noise
    Matrix<1, 1> kf_Sigma_z; // measurement noise
    Matrix<2, 2> kf_I2;

    // Initialize Kalman filter system model
    void initKalman(float sys_d, float sys_m, float proc_noise_std, float meas_noise_std)
    {
        kf_A = {0.0f, 1.0f, 0.0f, -sys_d / sys_m};
        kf_B = {0.0f, 1.0f / sys_m};
        kf_C = {1.0f, 0.0f};
        kf_Sigma_u = {proc_noise_std * proc_noise_std, 0.0f,
                      0.0f, proc_noise_std * proc_noise_std};
        kf_Sigma_z = {meas_noise_std * meas_noise_std};
        kf_I2 = {1.0f, 0.0f, 0.0f, 1.0f};
        kf_mu = {0.0f, 0.0f};
        kf_Sigma = {10000.0f, 0.0f, 0.0f, 10000.0f};
    }

    // Reset Kalman filter state to a known position
    void resetKalman(float initial_pos)
    {
        kf_mu(0, 0) = initial_pos;
        kf_mu(1, 0) = 0.0f;
        kf_Sigma = {10000.0f, 0.0f, 0.0f, 10000.0f};
    }

    // Kalman predict step (call every control loop)
    void kfPredict(float dt)
    {
        Matrix<2, 2> Ad = kf_I2 + kf_A * dt;
        Matrix<2, 1> Bd = kf_B * dt;
        float u_t = output_value / 100.0f;
        Matrix<1, 1> u_vec = {u_t};
        kf_mu = Ad * kf_mu + Bd * u_vec;
        kf_Sigma = Ad * kf_Sigma * (~Ad) + kf_Sigma_u;
    }

    // Kalman update step (call when new measurement arrives)
    void kfUpdate(float measurement)
    {
        Matrix<1, 1> y = {measurement};
        Matrix<1, 1> y_m = y - kf_C * kf_mu;
        Matrix<1, 1> S = kf_C * kf_Sigma * (~kf_C) + kf_Sigma_z;
        Matrix<1, 1> S_inv;
        S_inv(0, 0) = 1.0f / S(0, 0);
        Matrix<2, 1> K = kf_Sigma * (~kf_C) * S_inv;
        kf_mu = kf_mu + K * y_m;
        kf_Sigma = (kf_I2 - K * kf_C) * kf_Sigma;
    }

    // Get Kalman position estimate
    float kfPosition() { return kf_mu(0, 0); }
    // Get Kalman velocity estimate (useful as derivative)
    float kfVelocity() { return kf_mu(1, 0); }

    // Feed a new raw measurement for extrapolation tracking
    void feedMeasurement(float raw, unsigned long time_us)
    {
        if (extrap_last_time > 0)
        {
            float dt = (time_us - extrap_last_time) / 1.e6f;
            if (dt > 0)
                extrap_velocity = (raw - extrap_last_measurement) / dt;
        }
        extrap_last_measurement = raw;
        extrap_last_time = time_us;
    }

    // Get extrapolated estimate at current time
    float getExtrapolated(unsigned long current_time_us)
    {
        float dt = (current_time_us - extrap_last_time) / 1.e6f;
        return extrap_last_measurement + extrap_velocity * dt;
    }

    // Get the best sensor estimate based on sensor_mode.
    // For KALMAN: call kfPredict before this, then kfUpdate if has_new_measurement.
    // For EXTRAPOLATION: call feedMeasurement when new data arrives.
    // For DIRECT: pass raw measurement directly.
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

    void reset()
    {
        sensor_value = 0.0f;
        error_value = 0.0f;
        integral_value = 0.0f;
        derivative_value = 0.0f;
        raw_derivative_value = 0.0f;
        raw_integral_value = 0.0f;
        output_value = 0.0f;
        count = 0;
        extrap_velocity = 0.0f;
        extrap_last_measurement = 0.0f;
        extrap_last_time = 0;
    }

    // Core PID compute. Returns output clamped to [-output_limit, output_limit].
    // If wrap_angle is true, error and derivative use angle wrapping.
    // If use_kf_derivative is true, use Kalman velocity as derivative instead.
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

    static float wrapAngle180(float angle)
    {
        while (angle > 180.0f)
            angle -= 360.0f;
        while (angle < -180.0f)
            angle += 360.0f;
        return angle;
    }
};

// Distance PID — controls forward/backward via ToF
PIDController dist_pid;
// Orientation PID — controls turning via IMU yaw
PIDController orient_pid;
////////////////////////// PID Controller /////////////////////////

//////////// Commands ////////////
enum CommandTypes
{
    PING,
    START_RECORD,
    STOP_ROBOT,
    SEND_LOG,
    SET_DURATION,
    SET_MODE,
    SET_MOTOR_SCALE,
    UPDATE_DIST_PID,
    UPDATE_ORIENT_PID,
    SET_DIST_SETPOINT,
    SET_ORIENT_SETPOINT,
    SET_NAV_SETPOINTS,
    SET_DIST_SENSOR_MODE,
    SET_SAMPLE_RATE,
    SET_MAP_DEGREES,
    SET_NAV_TARGET,   // Args: heading_deg|distance_m|seg_id. START_RECORD fires it.
    SET_NAV_CALIB,    // Args: pwm|speed_mps
    RESET_YAW,        // No args — zeros relative yaw at current heading
    SET_NAV_DIST_MODE // Args: 0 (time-based) or 1 (KF-integrated)
};
//////////// Commands ////////////

//////////// Control Mode ////////////
enum ControlMode
{
    MODE_POSITION,
    MODE_ORIENTATION,
    MODE_RUSH,
    MODE_IDLE,
    MODE_FLIP,
    MODE_MAPPING,
    MODE_NAV_SEG
};

enum FlipState
{
    FLIP_READY,
    FLIP_STARTED,
    FLIP_RECOVER,
    FLIP_RETURN,
    FLIP_IDLE
};

enum MappingState
{
    MAP_START,
    MAP_TURN,
    MAP_STABILIZE,
    MAP_MEASURE,
    MAP_DONE
};

enum NavigationState
{
    NAV_IDLE,      // motors off, waiting for a new segment
    NAV_TURN,      // rotating in place to the target world-frame heading
    NAV_STABILIZE, // holding heading briefly so the IMU settles before driving
    NAV_GO,        // open-loop forward with orientation PID, time/ToF-bounded
    NAV_TAIL,      // motors off but controller still active so the coast-down
                   // is captured by collectSamples() before NAV_DONE freezes
    NAV_DONE       // snapshot final pose, deactivate, freeze buffers
};

ControlMode control_mode = MODE_IDLE;

FlipState flip_state = FLIP_READY;
unsigned long flip_time = 0; // in microseconds

MappingState map_state = MAP_START;
float map_increment = 20.0f;
float map_start_angle = 0.0f;
int map_step = 0;
unsigned long map_stabilize_time = 0;
float valid_map_tof1 = -1.0f;
float valid_map_tof2 = -1.0f;
float map_total_degrees = 360.0f;

// Lab 12 turn-go-turn segment executor state.
NavigationState nav_state = NAV_IDLE;
float nav_target_heading_deg = 0.0f;             // absolute world-frame heading (deg)
float nav_target_dist_m = 0.0f;                  // segment length (meters)
int nav_segment_id = 0;                          // echoed back in the done-notify
float nav_go_pwm = 70.0f;                        // open-loop forward PWM during NAV_GO
float nav_calib_speed_mps = 1.00f;               // calibrated forward speed at nav_go_pwm
float nav_safety_tof_mm = 200.0f;                // front-ToF threshold for safety stop
unsigned long nav_phase_start_us = 0;            // entered-current-state timestamp
float nav_kf_pos_start_mm = 0.0f;                // KF position at NAV_GO entry (mm)
float nav_dist_traveled_m = 0.0f;                // integrated distance during NAV_GO
const unsigned long NAV_STABILIZE_US = 200000UL; // 200 ms hold after turning
const unsigned long NAV_TAIL_US = 1000000UL;     // 1000 ms post-stop coast,
                                                 // collectSamples() keeps logging
const float NAV_TURN_TOL_DEG = 3.0f;             // |yaw err| to leave NAV_TURN
const float NAV_TIME_SAFETY_MULT = 1.0f;
const float NAV_KF_INIT_FALLBACK_MM = 5000.0f; // if no ToF target ahead

// Reason flag carried from NAV_GO into NAV_DONE so the ack reflects what
// actually stopped the segment.
const char *nav_stop_reason = "dist";

// One-shot: set true in NAV_GO when a stop fires, cleared after NAV_DONE
// snapshots the final pose. Ensures the snapshot runs exactly once.
bool nav_ack_pending = false;

// Latest readings captured at NAV_DONE entry. Python can pull these via the
// SEND_LOG buffer (the last buffered sample) or via Serial; no BLE notify
// is emitted from NAV_DONE itself.
float nav_final_tof_mm = 0.0f;
float nav_final_yaw_deg = 0.0f;

// Distance-control mode for NAV_GO. false = stop on calibrated elapsed time,
// true = stop on KF-integrated traveled distance. The non-primary check still
// runs at 3× expected time as a hard safety backup.
bool nav_use_kf_dist = false;

bool active = false;
bool tof1_updated = false;
bool tof2_updated = false;
bool wait_tof1_ready = false;
bool wait_tof2_ready = false;
bool imu_updated = false;
unsigned long last_control_time = 0; // in microseconds
//////////// Control Mode ////////////

// =========================
// SETUP
// =========================
void setup()
{
    Serial.begin(115200);

    // Default PID gains
    dist_pid.kp = 0.05f;
    dist_pid.ki = 0.0f;
    dist_pid.kd = 0.05f;
    dist_pid.sensor_mode = PIDController::EXTRAPOLATION;

    orient_pid.kp = 2.5f;
    orient_pid.ki = 1.2f;
    orient_pid.kd = 0.25f;
    orient_pid.sensor_mode = PIDController::DIRECT;

    // Initialize Kalman filter model for distance (drag/mass system)
    dist_pid.initKalman(
        0.000309f,  // sys_d (drag)
        10.579281f, // sys_m (mass)
        97.0f,      // process noise std
        15.0f       // measurement noise std
    );

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

    if (active)
    {
        bool should_run = false;
        bool dist_uses_prediction = (dist_pid.sensor_mode == PIDController::EXTRAPOLATION ||
                                     dist_pid.sensor_mode == PIDController::KALMAN);

        switch (control_mode)
        {
        case MODE_POSITION:
            should_run = tof2_updated || dist_uses_prediction;
            break;
        case MODE_ORIENTATION:
            should_run = imu_updated;
            break;
        case MODE_FLIP:
        case MODE_RUSH:
            should_run = tof2_updated || imu_updated || dist_uses_prediction;
            break;
        case MODE_NAV_SEG:
            // Run on any sensor tick so the FSM can advance through
            // NAV_TURN / NAV_STABILIZE / NAV_GO / NAV_DONE promptly.
            should_run = tof1_updated || tof2_updated || imu_updated || dist_uses_prediction;
            break;
        case MODE_IDLE:
        case MODE_MAPPING:
            should_run = tof1_updated || tof2_updated || imu_updated;
            break;
        }

        if (should_run)
        {
            runController();
            tof1_updated = false;
            tof2_updated = false;
            imu_updated = false;
        }
    }

    if (collecting)
        collectSamples();
    if (active && (millis() - start_sample_time >= SAMPLE_DURATION))
        stopRobot();
    if (!active)
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
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    success = robot_cmd.get_command_type(cmd_type);
    if (!success)
        return;

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

        // Initialize distance sensors for modes that use ToF
        if (control_mode == MODE_POSITION || control_mode == MODE_FLIP ||
            control_mode == MODE_RUSH || control_mode == MODE_IDLE ||
            control_mode == MODE_MAPPING || control_mode == MODE_NAV_SEG)
        {
            distanceSensor1.stopRanging();
            distanceSensor2.stopRanging();
            distanceSensor1.startRanging();
            distanceSensor2.startRanging();
            Serial.println("Waiting for ToF readings...");
            while (!distanceSensor1.checkForDataReady())
            {
                delay(1);
            }
            tof1_dist = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
            tof1_time = micros();
            tof1_updated = true;

            while (!distanceSensor2.checkForDataReady())
            {
                delay(1);
            }

            tof2_dist = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            tof2_time = micros();
            tof2_updated = true;

            if (control_mode == MODE_FLIP)
            {
                flip_state = (tof2_dist < dist_pid.setpoint) ? FLIP_IDLE : FLIP_READY;
            }

            dist_pid.sensor_value = tof2_dist;
            dist_pid.feedMeasurement(tof2_dist, tof2_time);
            dist_pid.resetKalman(tof2_dist);
        }

        // Initialize IMU for modes that use orientation
        if (control_mode == MODE_ORIENTATION || control_mode == MODE_IDLE ||
            control_mode == MODE_FLIP || control_mode == MODE_RUSH ||
            control_mode == MODE_MAPPING || control_mode == MODE_NAV_SEG)
        {
            myICM.resetFIFO();
            while (!updateIMU())
            {
                delay(1);
            }
            // Preserve the world-frame yaw across NAV_SEG segments — the
            // mission-start RESET_YAW already aligned IMU yaw with the
            // world heading, so re-zeroing here would make every segment
            // a fresh-relative turn (and accumulate world-frame drift).
            // All other modes still rezero on each START_RECORD.
            if (control_mode != MODE_NAV_SEG)
            {
                yaw_offset = dmp_yaw;
                continuous_yaw_offset = continuous_yaw;
            }
            gyr_z_offset = gyr_z;
            imu_updated = true;
            orient_pid.sensor_value = 0.0f;
            acc_x = 0.0f;
            acc_y = 0.0f;
            last_control_time = micros();
        }

        if (control_mode == MODE_MAPPING)
        {
            dist_pid.sensor_mode = PIDController::DIRECT;
            map_state = MAP_START;
            map_step = 0;
            Serial.println("Mapping Mode Initialized: Sensor mode forced to DIRECT.");
        }

        // Fire the turn-go-turn FSM. Targets must already be set via
        // SET_NAV_TARGET — START_RECORD is purely a "go" trigger here.
        if (control_mode == MODE_NAV_SEG)
        {
            orient_pid.reset();
            orient_pid.setpoint = nav_target_heading_deg;
            nav_state = NAV_TURN;
            nav_phase_start_us = micros();
            SAMPLE_DURATION = 15000; // 15 s hard safety cap
            Serial.print("NAV start seg=");
            Serial.print(nav_segment_id);
            Serial.print(" heading=");
            Serial.print(nav_target_heading_deg);
            Serial.print(" dist=");
            Serial.println(nav_target_dist_m);
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
        // Lab 12 fields (7 parts): T, LPWM, RPWM, AX, T2, DV, YW.
        // DV = dist_pid.sensor_value, which is loaded with dist_pid.kfPosition()
        // during NAV_GO so the log captures the KF distance estimate.
        for (int i = 0; i < sample_count; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append((int)time_buffer[i]);
            tx_estring_value.append("|LPWM: ");
            tx_estring_value.append(left_pwm[i]);
            tx_estring_value.append("|RPWM: ");
            tx_estring_value.append(right_pwm[i]);
            tx_estring_value.append("|AX: ");
            tx_estring_value.append(acc_x_buffer[i]);
            tx_estring_value.append("|T2: ");
            tx_estring_value.append(tof_2_buffer[i]);
            tx_estring_value.append("|DV: ");
            tx_estring_value.append(dist_sensor_buffer[i]);
            tx_estring_value.append("|YW: ");
            tx_estring_value.append(yaw_buffer[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(3);
        }
        tx_estring_value.clear();
        tx_estring_value.append("Sample Count: ");
        tx_estring_value.append(sample_count);
        tx_estring_value.append("| ToF1 Count: ");
        tx_estring_value.append(tof1_count);
        tx_estring_value.append("| ToF2 Count: ");
        tx_estring_value.append(tof2_count);
        tx_estring_value.append("| IMU Count: ");
        tx_estring_value.append(imu_count);
        tx_estring_value.append("| Dist PID Count: ");
        tx_estring_value.append(dist_pid.count);
        tx_estring_value.append("| Orient PID Count: ");
        tx_estring_value.append(orient_pid.count);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        break;
    }

    case UPDATE_DIST_PID:
    {
        float new_kp, new_ki, new_kd;
        success = robot_cmd.get_next_value(new_kp);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_ki);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_kd);
        if (!success)
            return;

        dist_pid.kp = new_kp;
        dist_pid.ki = new_ki;
        dist_pid.kd = new_kd;
        Serial.print("Set Dist PID: ");
        Serial.print(new_kp);
        Serial.print(", ");
        Serial.print(new_ki);
        Serial.print(", ");
        Serial.println(new_kd);
        break;
    }

    case UPDATE_ORIENT_PID:
    {
        float new_kp, new_ki, new_kd;
        success = robot_cmd.get_next_value(new_kp);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_ki);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_kd);
        if (!success)
            return;

        orient_pid.kp = new_kp;
        orient_pid.ki = new_ki;
        orient_pid.kd = new_kd;
        Serial.print("Set Orient PID: ");
        Serial.print(new_kp);
        Serial.print(", ");
        Serial.print(new_ki);
        Serial.print(", ");
        Serial.println(new_kd);
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

    case SET_DIST_SETPOINT:
    {
        float new_setpoint;
        success = robot_cmd.get_next_value(new_setpoint);
        if (!success)
            return;
        dist_pid.setpoint = new_setpoint;
        Serial.print("Set Dist Setpoint to: ");
        Serial.println(dist_pid.setpoint);
        break;
    }

    case SET_ORIENT_SETPOINT:
    {
        float new_setpoint;
        success = robot_cmd.get_next_value(new_setpoint);
        if (!success)
            return;
        orient_pid.setpoint = PIDController::wrapAngle180(new_setpoint);
        Serial.print("Set Orient Setpoint to: ");
        Serial.println(orient_pid.setpoint);
        break;
    }

    case SET_NAV_SETPOINTS:
    {
        float new_dist_sp, new_orient_sp;
        success = robot_cmd.get_next_value(new_dist_sp);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_orient_sp);
        if (!success)
            return;
        dist_pid.setpoint = new_dist_sp;
        orient_pid.setpoint = PIDController::wrapAngle180(new_orient_sp);
        Serial.print("Set Nav Setpoints: dist=");
        Serial.print(dist_pid.setpoint);
        Serial.print(", orient=");
        Serial.println(orient_pid.setpoint);
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
        case MODE_MAPPING:
            control_mode = MODE_MAPPING;
            map_state = MAP_START;
            map_step = 0;
            dist_pid.sensor_mode = PIDController::DIRECT;
            Serial.println("Starting Orientation Mapping...");
            break;
        case MODE_NAV_SEG:
            control_mode = MODE_NAV_SEG;
            nav_state = NAV_IDLE;
            setMotors(0.0f, 0.0f);
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

    // 0 = DIRECT, 1 = EXTRAPOLATION, 2 = KALMAN
    case SET_DIST_SENSOR_MODE:
    {
        int mode_int;
        success = robot_cmd.get_next_value(mode_int);
        if (!success)
            return;
        switch (mode_int)
        {
        case 0:
            dist_pid.sensor_mode = PIDController::DIRECT;
            Serial.println("Dist sensor mode: DIRECT");
            break;
        case 1:
            dist_pid.sensor_mode = PIDController::EXTRAPOLATION;
            Serial.println("Dist sensor mode: EXTRAPOLATION");
            break;
        case 2:
            dist_pid.sensor_mode = PIDController::KALMAN;
            Serial.println("Dist sensor mode: KALMAN");
            break;
        default:
            Serial.print("Invalid sensor mode: ");
            Serial.println(mode_int);
        }
        break;
    }

    case SET_SAMPLE_RATE:
    {
        int new_rate;
        success = robot_cmd.get_next_value(new_rate);
        if (!success)
            return;
        SAMPLE_INTERVAL = 1000000 / new_rate;
        Serial.print("Set Sample Rate to: ");
        Serial.print(new_rate);
        Serial.print(" Hz (Interval: ");
        Serial.print(SAMPLE_INTERVAL);
        Serial.println(" us)");
        break;
    }

    case SET_MAP_DEGREES:
    {
        float new_deg;
        success = robot_cmd.get_next_value(new_deg);
        if (!success)
            return;
        map_total_degrees = new_deg;
        Serial.print("Set Mapping Degrees to: ");
        Serial.println(map_total_degrees);
        break;
    }

    case SET_NAV_TARGET:
    {
        // Args: heading_deg | distance_m | seg_id
        // Just records the target. START_RECORD is what actually fires the
        // segment — it re-runs the ToF + IMU init path before turning on the
        // controller, so the sensors come up cleanly between runs.
        float heading_deg, distance_m;
        int seg_id;
        success = robot_cmd.get_next_value(heading_deg);
        if (!success)
            return;
        success = robot_cmd.get_next_value(distance_m);
        if (!success)
            return;
        success = robot_cmd.get_next_value(seg_id);
        if (!success)
            return;

        nav_target_heading_deg = PIDController::wrapAngle180(heading_deg);
        nav_target_dist_m = distance_m;
        nav_segment_id = seg_id;

        Serial.print("NAV target seg=");
        Serial.print(seg_id);
        Serial.print(" heading=");
        Serial.print(nav_target_heading_deg);
        Serial.print(" dist=");
        Serial.println(nav_target_dist_m);
        break;
    }

    case SET_NAV_CALIB:
    {
        // Args: pwm | speed_mps
        float new_pwm, new_speed;
        success = robot_cmd.get_next_value(new_pwm);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_speed);
        if (!success)
            return;
        nav_go_pwm = new_pwm;
        nav_calib_speed_mps = new_speed;
        Serial.print("Nav calib: pwm=");
        Serial.print(nav_go_pwm);
        Serial.print(" speed=");
        Serial.println(nav_calib_speed_mps);
        break;
    }

    case RESET_YAW:
    {
        // Zero the relative-yaw reference at the current heading. After this,
        // getOrientationSensorValue() returns 0 until the robot rotates.
        continuous_yaw_offset = continuous_yaw;
        orient_pid.reset();
        Serial.print("Yaw zeroed at raw=");
        Serial.println(continuous_yaw);
        break;
    }

    case SET_NAV_DIST_MODE:
    {
        int mode_int;
        success = robot_cmd.get_next_value(mode_int);
        if (!success)
            return;
        nav_use_kf_dist = (mode_int != 0);
        Serial.print("Nav dist mode: ");
        Serial.println(nav_use_kf_dist ? "KF-integrated" : "time-based");
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

// Get orientation sensor value (yaw relative to offset)
float getOrientationSensorValue()
{
    return continuous_yaw - continuous_yaw_offset;
}

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
        float estimate = dist_pid.getEstimate(tof2_dist, tof2_updated, dt);
        bool use_kf_d = (dist_pid.sensor_mode == PIDController::KALMAN);
        dist_pid.compute(estimate, dt, false, use_kf_d);
        applyLinearOutput(dist_pid.output_value);
        break;
    }

    case MODE_ORIENTATION:
    {
        float new_sensor = getOrientationSensorValue();
        orient_pid.compute(new_sensor, dt, true);
        // Negate: positive error -> turn one way
        applyAngularOutput(-orient_pid.output_value);
        break;
    }

    case MODE_NAV_SEG:
    {
        // Lab 12 turn-go-turn segment executor.
        float orient_sensor = getOrientationSensorValue();

        switch (nav_state)
        {
        case NAV_IDLE:
        {
            setMotors(0.0f, 0.0f);
            break;
        }

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

        case NAV_STABILIZE:
        {
            orient_pid.setpoint = nav_target_heading_deg;
            float angular = orient_pid.compute(orient_sensor, dt, true);
            applyAngularOutput(-angular);
            if (current_control_time - nav_phase_start_us > NAV_STABILIZE_US)
            {
                setMotors(0.0f, 0.0f);
                // Seed the Kalman filter with the current ToF reading so we
                // can read out integrated traveled distance during NAV_GO.
                // Fall back to a large constant if the ToF is invalid so that
                // start − now still produces a positive traveled distance.
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

        case NAV_GO:
        {
            // Open-loop forward at the calibrated PWM, with orientation PID
            // mixed into left/right to keep the heading constant
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

            dist_pid.output_value = nav_go_pwm;
            dist_pid.kfPredict(dt);
            if (tof2_updated && tof2_dist > 10.0f && tof2_dist < 6000.0f)
            {
                dist_pid.kfUpdate(tof2_dist);
            }
            dist_pid.sensor_value = dist_pid.kfPosition();
            // Position-delta integration: kfPosition tracks ToF tightly, so
            // (start - now) is the actual forward travel. Replaces the old
            // fabs(kfVelocity)*dt integral, which underestimated travel
            // because the KF velocity is biased toward zero (model input
            // pushes v positive while ToF pulls position negative).
            nav_dist_traveled_m =
                fabs(nav_kf_pos_start_mm - dist_pid.kfPosition()) / 1000.0f;

            // Stop conditions:
            //   primary_stop : EITHER the picked mode fires OR the 1× time
            //                  safety fires — whichever comes first
            //   backup_stop  : 3× expected time, hard cap if everything else
            //                  is broken
            //   safety_stop  : front ToF reads below the collision threshold
            float elapsed_s = (current_control_time - nav_phase_start_us) / 1.0e6f;
            float expected_s = (nav_calib_speed_mps > 1e-3f)
                                   ? nav_target_dist_m / nav_calib_speed_mps
                                   : 0.0f;
            bool dist_done = nav_dist_traveled_m >= nav_target_dist_m;
            bool time_done = elapsed_s >= NAV_TIME_SAFETY_MULT * expected_s;
            bool safety_stop = (tof2_dist > 0.0f) && (tof2_dist < nav_safety_tof_mm);

            // KF mode trusts its position-delta integrator; the 1× time
            // gate is intentionally NOT applied here because v_calib is
            // the steady-state speed, not the average over a from-rest
            // segment, so it would clip the run before the robot has
            // actually traveled the commanded distance. The 3× time
            // backup below still protects against a stuck integrator.
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
                    nav_stop_reason = "backup"; // primary didn't fire in time

                // Hand off to NAV_TAIL — motors stay off, but the controller
                // keeps running so collectSamples() captures the coast-down
                // before NAV_DONE freezes the buffers.
                nav_ack_pending = true;
                nav_state = NAV_TAIL;
                nav_phase_start_us = current_control_time;
            }
            break;
        }

        case NAV_TAIL:
        {
            // Motors already off. Wait the tail window so the post-stop
            // trajectory ends up in the SEND_LOG stream.
            setMotors(0.0f, 0.0f);
            if (current_control_time - nav_phase_start_us > NAV_TAIL_US)
            {
                nav_state = NAV_DONE;
                nav_phase_start_us = current_control_time;
            }
            break;
        }

        case NAV_DONE:
        {
            setMotors(0.0f, 0.0f);
            if (nav_ack_pending)
            {
                nav_final_tof_mm = tof2_dist;
                nav_final_yaw_deg = getOrientationSensorValue();

                // 4-field DONE notify (distinct from the 7-field log samples).
                // Python parses these in the `len(parts) == 4` branch.
                //   D: <seg> | S: <reason> | F: <final tof, mm> | Y: <final yaw, deg>
                tx_estring_value.clear();
                tx_estring_value.append("D: ");
                tx_estring_value.append((int)nav_segment_id);
                tx_estring_value.append("|S: ");
                tx_estring_value.append(nav_stop_reason);
                tx_estring_value.append("|F: ");
                tx_estring_value.append(nav_final_tof_mm);
                tx_estring_value.append("|Y: ");
                tx_estring_value.append(nav_final_yaw_deg);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                active = false;     // halt runController
                collecting = false; // freeze the sample buffers
                nav_ack_pending = false;
                digitalWrite(LED_BUILTIN, LOW);
            }
            break;
        }
        }
        break;
    }

    case MODE_RUSH:
    {
        float estimate = dist_pid.getEstimate(tof2_dist, tof2_updated, dt);
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

    case MODE_IDLE:
    {
        dist_pid.output_value = 0.0f;
        orient_pid.output_value = 0.0f;
        applyLinearOutput(0.0f);
        break;
    }

    case MODE_FLIP:
    {
        bool valid_tof = tof2_updated;
        if (flip_state == FLIP_STARTED || flip_state == FLIP_RECOVER)
            valid_tof = false;

        float dist_estimate = dist_pid.getEstimate(tof2_dist, valid_tof, dt);
        dist_pid.sensor_value = dist_estimate;

        float orient_sensor = getOrientationSensorValue();

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
        else if (flip_state == FLIP_RETURN)
        {
            dist_pid.output_value = -100.0f;
            float angular = -orient_pid.compute(orient_sensor, dt, true);
            left_motor_pct = constrain(-(100.0f - angular) * MOTOR_SCALE, -100.0f, 100.0f);
            right_motor_pct = constrain(-(100.0f + angular) * MOTOR_SCALE, -100.0f, 100.0f);
            setMotors(left_motor_pct, right_motor_pct);
        }

        break;
    }

    case MODE_MAPPING:
    {
        float orient_sensor = getOrientationSensorValue();

        switch (map_state)
        {
        case MAP_START:
            map_start_angle = orient_sensor;
            map_step = 1;
            orient_pid.reset();
            map_state = MAP_TURN;
            break;

        case MAP_TURN:
        {
            float target_angle = map_start_angle + (map_step * map_increment);
            orient_pid.setpoint = target_angle;

            float angular = orient_pid.compute(orient_sensor, dt, false);
            applyAngularOutput(-angular);

            if (abs(orient_pid.error_value) < 3.0f)
            {
                map_state = MAP_STABILIZE;
                map_stabilize_time = current_control_time;
            }
            break;
        }

        case MAP_STABILIZE:
        {
            float target_angle = map_start_angle + (map_step * map_increment);
            orient_pid.setpoint = target_angle;
            float angular = orient_pid.compute(orient_sensor, dt, false);
            applyAngularOutput(-angular);

            if (current_control_time - map_stabilize_time > 500000)
            {
                applyAngularOutput(0.0f);
                wait_tof1_ready = false;
                wait_tof2_ready = false;
                map_state = MAP_MEASURE;
            }
            break;
        }

        case MAP_MEASURE:
        {
            applyAngularOutput(0.0f);
            if (tof1_updated)
                wait_tof1_ready = true;
            if (tof2_updated)
                wait_tof2_ready = true;

            if (wait_tof1_ready && wait_tof2_ready)
            {
                valid_map_tof1 = tof1_dist;
                valid_map_tof2 = tof2_dist;
                map_step++;

                wait_tof1_ready = false;
                wait_tof2_ready = false;

                if (map_step * map_increment >= map_total_degrees)
                    map_state = MAP_DONE;

                else
                    map_state = MAP_TURN;
            }
            break;
        }

        case MAP_DONE:
            orient_pid.setpoint = map_start_angle + map_total_degrees;
            float angular = orient_pid.compute(orient_sensor, dt, false);
            applyAngularOutput(-angular);
            if (abs(orient_pid.error_value) < 5.0f || (millis() - start_sample_time >= SAMPLE_DURATION))
            {
                if (tof1_updated && tof2_updated)
                {
                    applyAngularOutput(0.0f);
                    valid_map_tof1 = tof1_dist;
                    valid_map_tof2 = tof2_dist;
                    active = false;
                }
            }
            break;
        }
        break;
    }

    default:
        break;
    }
}

// Apply output as linear (both motors same direction)
void applyLinearOutput(float output)
{
    float power = constrain(output * MOTOR_SCALE, -100.0f, 100.0f);
    left_motor_pct = power;
    right_motor_pct = power;
    setMotors(power, power);
}

// Apply output as angular (motors opposite direction)
void applyAngularOutput(float output)
{
    float power = constrain(output * MOTOR_SCALE, -100.0f, 100.0f);
    left_motor_pct = power;
    right_motor_pct = -power;
    setMotors(power, -power);
}

// =========================
// Sensors
// =========================
void updateSensors()
{
    if (updateIMU())
    {
        imu_updated = true;
    }

    if (distanceSensor1.checkForDataReady())
    {
        tof1_dist = distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        tof1_time = micros();
        tof1_count++;
        tof1_updated = true;
    }

    if (distanceSensor2.checkForDataReady())
    {
        tof2_dist = distanceSensor2.getDistance();
        distanceSensor2.clearInterrupt();
        tof2_time = micros();
        tof2_count++;
        tof2_updated = true;

        // Feed measurement to distance controller for extrapolation tracking
        dist_pid.feedMeasurement(tof2_dist, tof2_time);
    }
}

bool updateIMU()
{
    if (myICM.dataReady())
    {
        myICM.getAGMT();
        acc_x = myICM.accX();
        acc_y = myICM.accY();
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
            // dmp_pitch = asin(2.0 * (q0 * q2 - q3 * q1)) * 180.0 / PI;
            dmp_roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * 180.0 / PI;

            float delta_yaw = dmp_yaw - prev_raw_yaw;
            if (delta_yaw > 180.0f)
                delta_yaw -= 360.0f;
            else if (delta_yaw < -180.0f)
                delta_yaw += 360.0f;

            continuous_yaw += delta_yaw;
            prev_raw_yaw = dmp_yaw;

            imu_count++;

            return true;
        }
    }
    return false;
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
    if (control_mode == MODE_MAPPING)
    {
        tof_1_buffer[sample_count] = valid_map_tof1;
        tof_2_buffer[sample_count] = valid_map_tof2;
        valid_map_tof1 = -1.0f;
        valid_map_tof2 = -1.0f;
    }
    else
    {
        tof_1_buffer[sample_count] = tof1_dist;
        tof_2_buffer[sample_count] = tof2_dist;
    }
    acc_x_buffer[sample_count] = acc_x;
    // acc_y_buffer[sample_count] = acc_y;
    gyr_z_buffer[sample_count] = gyr_z;
    yaw_buffer[sample_count] = dmp_yaw;
    // pitch_buffer[sample_count] = dmp_pitch;
    roll_buffer[sample_count] = dmp_roll;

    // Motor Buffer
    left_pwm[sample_count] = (left_motor_pct > 0 ? 1 : -1) * percentToPWM(left_motor_pct, true);
    right_pwm[sample_count] = (right_motor_pct > 0 ? 1 : -1) * percentToPWM(right_motor_pct, false);
    left_percent[sample_count] = left_motor_pct;
    right_percent[sample_count] = right_motor_pct;

    // Distance PID Buffer
    dist_setpoint_buffer[sample_count] = dist_pid.setpoint;
    dist_sensor_buffer[sample_count] = dist_pid.sensor_value;
    dist_output_buffer[sample_count] = dist_pid.output_value;

    // Orientation PID Buffer
    orient_setpoint_buffer[sample_count] = orient_pid.setpoint;
    orient_sensor_buffer[sample_count] = orient_pid.sensor_value;
    orient_output_buffer[sample_count] = orient_pid.output_value;
    // orient_error_buffer[sample_count] = orient_pid.error_value;
    // orient_integral_buffer[sample_count] = orient_pid.integral_value;
    // orient_derivative_buffer[sample_count] = orient_pid.derivative_value;

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
        acc_x_buffer[i] = 0.0f;
        // acc_y_buffer[i] = 0.0f;
        gyr_z_buffer[i] = 0.0f;
        yaw_buffer[i] = 0.0f;
        // pitch_buffer[i] = 0.0f;
        // roll_buffer[i] = 0.0f;
        tof_1_buffer[i] = 0.0f;
        tof_2_buffer[i] = 0.0f;
        dist_setpoint_buffer[i] = 0.0f;
        dist_sensor_buffer[i] = 0.0f;
        dist_output_buffer[i] = 0.0f;
        orient_setpoint_buffer[i] = 0.0f;
        orient_sensor_buffer[i] = 0.0f;
        orient_output_buffer[i] = 0.0f;
        // orient_error_buffer[i] = 0.0f;
        // orient_integral_buffer[i] = 0.0f;
        // orient_derivative_buffer[i] = 0.0f;
    }
}

void cleanState()
{
    // IMU
    acc_x = 0.0f;
    acc_y = 0.0f;
    gyr_z = 0.0f;
    dmp_yaw = 0.0f;
    gyr_z_offset = 0.0f;
    imu_count = 0;

    // TOF
    tof1_dist = 0.0f;
    tof2_dist = 0.0f;
    tof1_count = 0;
    tof2_count = 0;
    tof1_time = 0;
    tof2_time = 0;

    // Motors
    setMotors(0, 0);
    left_motor_pct = 0.0f;
    right_motor_pct = 0.0f;

    // Controllers
    dist_pid.reset();
    orient_pid.reset();

    active = false;
    tof1_updated = false;
    tof2_updated = false;
    wait_tof1_ready = false;
    wait_tof2_ready = false;

    imu_updated = false;
    last_control_time = 0;

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
    if (control_mode == MODE_POSITION || control_mode == MODE_FLIP ||
        control_mode == MODE_RUSH || control_mode == MODE_IDLE ||
        control_mode == MODE_MAPPING)
    {
        distanceSensor1.stopRanging();
        distanceSensor2.stopRanging();
    }
    active = false;
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("Stopped Robot");
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
            initialized = true;
        }
    }

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

void setupToF()
{
    // Turn off Sensor 2 to prevent I2C address conflicts
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(10);

    // Initialize Sensor 1
    while (distanceSensor1.begin(WIRE_PORT) != 0)
    {
        SERIAL_PORT.println("ToF Sensor 1 failed to begin. Retrying in 500ms...");
        delay(500);
    }

    // Change Sensor 1's I2C address (Default is 0x29, we change it to 0x2A)
    distanceSensor1.setI2CAddress(0x2A << 1);

    // Turn on Sensor 2
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);

    // Initialize Sensor 2
    while (distanceSensor2.begin(WIRE_PORT) != 0)
    {
        SERIAL_PORT.println("ToF Sensor 2 failed to begin. Retrying in 500ms...");
        delay(500);
    }

    distanceSensor1.setDistanceModeLong();
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
