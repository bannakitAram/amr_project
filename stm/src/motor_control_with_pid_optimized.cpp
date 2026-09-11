#include <Arduino.h>
#include <math.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <nav_msgs/msg/odometry.h>


// ============================================================
// V9.13
// STM32F407VET6
//
// Based on known-working V9 structure.
//
// Added:
//   1. DB3 calibrated feed-forward
//   2. Startup left/right balance
//   3. Gentle steady-state left/right matching
//   4. PI anti-windup
//
// Hardware structure is kept:
//
// RIGHT
//   SP   = PA5
//   FR   = PB0
//   RUN  = PC7
//   HAL  = PD11
//
// LEFT
//   SP   = PA4
//   FR   = PB1
//   RUN  = PC6
//   HAL  = PD8
// ============================================================


// ============================================================
// PIN CONFIGURATION
// ============================================================

#define R_SP   PA5
#define R_FR   PB0
#define R_RUN  PC7
#define R_HAL  PD11

#define L_SP   PA4
#define L_FR   PB1
#define L_RUN  PC6
#define L_HAL  PD8


// ============================================================
// ROBOT PARAMETERS
// ============================================================

const float WHEEL_RADIUS = 0.128f;
const float WHEEL_SEP    = 0.472f;

const float MAX_SPEED_MS = 1.9f;


// ============================================================
// ENCODER CALIBRATION
//
// V9 measured calibration
// ============================================================

const float PPR_L = 619.05f;
const float PPR_R = 550.53f;


// ============================================================
// CONTROL
// ============================================================

const uint32_t CONTROL_PERIOD_MS = 50;
const uint32_t VELOCITY_WINDOW_MS = 100;


// ============================================================
// FILTER
// ============================================================

const float CONTROL_ALPHA_L = 0.15f;
const float CONTROL_ALPHA_R = 0.15f;

const float ODOM_ALPHA_L = 0.15f;
const float ODOM_ALPHA_R = 0.15f;


// ============================================================
// PI
//
// Keep original V9 gains.
// FF calibration should remove most of the error.
// ============================================================

const float KP_L = 550.0f;
const float KI_L = 260.0f;

const float KP_R = 500.0f;
const float KI_R = 220.0f;


// ============================================================
// PI LIMITS
// ============================================================

const float INTEGRAL_LIMIT = 0.60f;

const float PI_LIMIT = 220.0f;


// ============================================================
// INTEGRAL CONDITIONS
// ============================================================

const float INTEGRAL_ERROR_LIMIT = 0.20f;
const float INTEGRAL_MIN_SPEED   = 0.05f;


// ============================================================
// STOP
// ============================================================

const float STOP_CMD_THRESHOLD   = 0.003f;
const float STOP_SPEED_THRESHOLD = 0.035f;


// ============================================================
// DAC
// ============================================================

const float DAC_MIN = 0.0f;
const float DAC_MAX = 4095.0f;


// ============================================================
// DAC SLEW
// ============================================================

const float DAC_SLEW = 20.0f;
const float DAC_SLEW_STOP = 60.0f;


// ============================================================
// ACCELERATION
// ============================================================

const float MAX_ACCEL = 0.50f;
const float MAX_DECEL = 1.20f;


// ============================================================
// COMMAND FILTER
// ============================================================

const float CMD_ALPHA = 0.20f;


// ============================================================
// DB3 CALIBRATION
//
// Steady-state measured values
//
// speed      LEFT DAC      RIGHT DAC
//
// 0.10         386           467
// 0.20         716           827
// 0.30        1044          1202
// 0.40        1373          1594
// 0.50        1694          1954
// 0.60        2021          2284
// 0.70        2348          2528
// 0.80        2668          2829
// 0.90        2999          3178
// 1.00        3325          3526
//
// Interpolation is used.
// ============================================================

#define CAL_POINTS 10

const float CAL_SPEED[CAL_POINTS] =
{
    0.10f,
    0.20f,
    0.30f,
    0.40f,
    0.50f,
    0.60f,
    0.70f,
    0.80f,
    0.90f,
    1.00f
};

const float CAL_DAC_L[CAL_POINTS] =
{
    386.0f,
    716.0f,
    1044.0f,
    1373.0f,
    1694.0f,
    2021.0f,
    2348.0f,
    2668.0f,
    2999.0f,
    3325.0f
};

const float CAL_DAC_R[CAL_POINTS] =
{
    467.0f,
    827.0f,
    1202.0f,
    1594.0f,
    1954.0f,
    2284.0f,
    2528.0f,
    2829.0f,
    3178.0f,
    3526.0f
};


// ============================================================
// STARTUP BALANCE
//
// Only active during startup.
// Does NOT continuously fight the PI.
// ============================================================

const uint32_t STARTUP_BALANCE_TIME = 500;

const float STARTUP_MIN_SPEED = 0.10f;

const float STARTUP_GAIN = 0.20f;

const float STARTUP_MAX_CORRECTION = 0.06f;


// ============================================================
// STEADY LEFT/RIGHT MATCH
//
// Very small correction.
//
// Right was historically slightly faster.
// Do not use aggressive K_STRAIGHT.
// ============================================================

const float MATCH_DEADBAND = 0.004f;

const float MATCH_GAIN = 0.05f;

const float MATCH_MAX_CORRECTION = 0.025f;


// ============================================================
// ROS
// ============================================================

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;


// ============================================================
// SUBSCRIBERS
// ============================================================

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t left_cmd_sub;
rcl_subscription_t right_cmd_sub;
rcl_subscription_t pulse_reset_sub;


// ============================================================
// PUBLISHERS
// ============================================================

rcl_publisher_t left_speed_pub;
rcl_publisher_t right_speed_pub;

rcl_publisher_t left_dac_pub;
rcl_publisher_t right_dac_pub;

rcl_publisher_t odom_pub;


// ============================================================
// MESSAGES
// ============================================================

geometry_msgs__msg__Twist cmd_vel_msg;

std_msgs__msg__Float32 left_cmd_msg;
std_msgs__msg__Float32 right_cmd_msg;

std_msgs__msg__Bool pulse_reset_msg;

std_msgs__msg__Float32 left_speed_msg;
std_msgs__msg__Float32 right_speed_msg;

std_msgs__msg__Float32 left_dac_msg;
std_msgs__msg__Float32 right_dac_msg;

nav_msgs__msg__Odometry odom_msg;


// ============================================================
// ENCODER
// ============================================================

volatile int32_t left_pulses = 0;
volatile int32_t right_pulses = 0;


// ============================================================
// VELOCITY
// ============================================================

float left_speed_raw = 0.0f;
float right_speed_raw = 0.0f;

float left_speed_control = 0.0f;
float right_speed_control = 0.0f;

float left_speed_odom = 0.0f;
float right_speed_odom = 0.0f;


// ============================================================
// COMMAND
// ============================================================

float cmd_linear = 0.0f;
float cmd_angular = 0.0f;

float cmd_linear_filtered = 0.0f;
float cmd_angular_filtered = 0.0f;

float target_left = 0.0f;
float target_right = 0.0f;

float target_left_limited = 0.0f;
float target_right_limited = 0.0f;


// ============================================================
// CONTROLLER STATE
// ============================================================

float integral_left = 0.0f;
float integral_right = 0.0f;

float left_dac_output = 0.0f;
float right_dac_output = 0.0f;


// ============================================================
// STARTUP STATE
// ============================================================

bool startup_active = false;
uint32_t startup_time = 0;


// ============================================================
// ODOM
// ============================================================

float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_yaw = 0.0f;


// ============================================================
// TIME
// ============================================================

uint32_t last_control_ms = 0;
uint32_t last_velocity_ms = 0;

int32_t last_left_pulses = 0;
int32_t last_right_pulses = 0;


// ============================================================
// UTILITY
// ============================================================

float clampf(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}


// ============================================================
// ENCODER ISR
//
// Keep encoder direction logic simple.
// Pulse counting itself is preserved.
// Direction is determined from FR pin.
// ============================================================

void leftEncoderISR()
{
    if (digitalRead(L_FR))
        left_pulses++;
    else
        left_pulses--;
}


void rightEncoderISR()
{
    if (digitalRead(R_FR))
        right_pulses++;
    else
        right_pulses--;
}


// ============================================================
// DAC
// ============================================================

void writeLeftDAC(float value)
{
    value =
        clampf(
            value,
            DAC_MIN,
            DAC_MAX);

    analogWrite(
        L_SP,
        (int)value);
}


void writeRightDAC(float value)
{
    value =
        clampf(
            value,
            DAC_MIN,
            DAC_MAX);

    analogWrite(
        R_SP,
        (int)value);
}


// ============================================================
// DAC SLEW
// ============================================================

float applySlew(
    float current,
    float target,
    float step)
{
    if (target > current)
    {
        current += step;

        if (current > target)
            current = target;
    }
    else
    {
        current -= step;

        if (current < target)
            current = target;
    }

    return current;
}


// ============================================================
// CALIBRATED DAC INTERPOLATION
// ============================================================

float calibratedDAC(
    float speed,
    const float *table)
{
    speed = fabsf(speed);

    if (speed <= 0.0f)
        return 0.0f;

    if (speed <= CAL_SPEED[0])
        return table[0];

    if (speed >= CAL_SPEED[CAL_POINTS - 1])
        return table[CAL_POINTS - 1];

    for (int i = 0; i < CAL_POINTS - 1; i++)
    {
        if (speed >= CAL_SPEED[i] &&
            speed <= CAL_SPEED[i + 1])
        {
            float ratio =
                (speed - CAL_SPEED[i]) /
                (CAL_SPEED[i + 1] - CAL_SPEED[i]);

            return
                table[i] +
                ratio *
                (table[i + 1] - table[i]);
        }
    }

    return table[CAL_POINTS - 1];
}


// ============================================================
// LEFT FF
// ============================================================

float leftFeedForward(float speed)
{
    float abs_speed = fabsf(speed);

    if (abs_speed < STARTUP_MIN_SPEED)
        return 0.0f;

    return calibratedDAC(
        abs_speed,
        CAL_DAC_L);
}


// ============================================================
// RIGHT FF
// ============================================================

float rightFeedForward(float speed)
{
    float abs_speed = fabsf(speed);

    if (abs_speed < STARTUP_MIN_SPEED)
        return 0.0f;

    return calibratedDAC(
        abs_speed,
        CAL_DAC_R);
}


// ============================================================
// VELOCITY ESTIMATION
// ============================================================

void updateVelocity()
{
    uint32_t now = millis();

    uint32_t elapsed =
        now - last_velocity_ms;

    if (elapsed < VELOCITY_WINDOW_MS)
        return;

    float dt =
        elapsed / 1000.0f;

    last_velocity_ms = now;


    int32_t lp;
    int32_t rp;

    noInterrupts();

    lp = left_pulses;
    rp = right_pulses;

    interrupts();


    int32_t dL =
        lp - last_left_pulses;

    int32_t dR =
        rp - last_right_pulses;


    last_left_pulses = lp;
    last_right_pulses = rp;


    // --------------------------------------------------------
    // LEFT
    // --------------------------------------------------------

    float left_revolutions =
        ((float)dL) / PPR_L;

    float left_distance =
        left_revolutions *
        (2.0f * PI * WHEEL_RADIUS);

    left_speed_raw =
        left_distance / dt;


    // --------------------------------------------------------
    // RIGHT
    // --------------------------------------------------------

    float right_revolutions =
        ((float)dR) / PPR_R;

    float right_distance =
        right_revolutions *
        (2.0f * PI * WHEEL_RADIUS);

    right_speed_raw =
        right_distance / dt;


    // --------------------------------------------------------
    // CONTROL FILTER
    // --------------------------------------------------------

    left_speed_control =
        CONTROL_ALPHA_L *
        left_speed_raw +
        (1.0f - CONTROL_ALPHA_L) *
        left_speed_control;


    right_speed_control =
        CONTROL_ALPHA_R *
        right_speed_raw +
        (1.0f - CONTROL_ALPHA_R) *
        right_speed_control;


    // --------------------------------------------------------
    // ODOM FILTER
    // --------------------------------------------------------

    left_speed_odom =
        ODOM_ALPHA_L *
        left_speed_raw +
        (1.0f - ODOM_ALPHA_L) *
        left_speed_odom;


    right_speed_odom =
        ODOM_ALPHA_R *
        right_speed_raw +
        (1.0f - ODOM_ALPHA_R) *
        right_speed_odom;
}


// ============================================================
// CMD VEL
// ============================================================

void cmdVelCallback(
    const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    cmd_linear =
        clampf(
            msg->linear.x,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);

    cmd_angular =
        msg->angular.z;
}


// ============================================================
// DIRECT LEFT
// ============================================================

void leftCmdCallback(
    const void *msgin)
{
    const std_msgs__msg__Float32 *msg =
        (const std_msgs__msg__Float32 *)msgin;

    target_left =
        clampf(
            msg->data,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);
}


// ============================================================
// DIRECT RIGHT
// ============================================================

void rightCmdCallback(
    const void *msgin)
{
    const std_msgs__msg__Float32 *msg =
        (const std_msgs__msg__Float32 *)msgin;

    target_right =
        clampf(
            msg->data,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);
}


// ============================================================
// RESET
// ============================================================

void resetSpeedController()
{
    integral_left = 0.0f;
    integral_right = 0.0f;

    left_dac_output = 0.0f;
    right_dac_output = 0.0f;

    target_left_limited = 0.0f;
    target_right_limited = 0.0f;

    left_speed_control = 0.0f;
    right_speed_control = 0.0f;

    noInterrupts();

    left_pulses = 0;
    right_pulses = 0;

    interrupts();

    last_left_pulses = 0;
    last_right_pulses = 0;
}


void pulseResetCallback(
    const void *msgin)
{
    const std_msgs__msg__Bool *msg =
        (const std_msgs__msg__Bool *)msgin;

    if (msg->data)
        resetSpeedController();
}


// ============================================================
// WHEEL TARGET
// ============================================================

void calculateWheelTarget()
{
    cmd_linear_filtered =
        CMD_ALPHA *
        cmd_linear +
        (1.0f - CMD_ALPHA) *
        cmd_linear_filtered;


    cmd_angular_filtered =
        CMD_ALPHA *
        cmd_angular +
        (1.0f - CMD_ALPHA) *
        cmd_angular_filtered;


    target_left =
        cmd_linear_filtered -
        cmd_angular_filtered *
        WHEEL_SEP *
        0.5f;


    target_right =
        cmd_linear_filtered +
        cmd_angular_filtered *
        WHEEL_SEP *
        0.5f;


    target_left =
        clampf(
            target_left,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);

    target_right =
        clampf(
            target_right,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);
}


// ============================================================
// TARGET RAMP
// ============================================================

float rampTarget(
    float current,
    float target,
    float dt)
{
    float delta =
        target - current;


    float max_up =
        MAX_ACCEL * dt;

    float max_down =
        MAX_DECEL * dt;


    if (delta > max_up)
        delta = max_up;

    if (delta < -max_down)
        delta = -max_down;


    return current + delta;
}


// ============================================================
// STARTUP BALANCE
//
// If right starts faster than left:
//   increase left slightly
//   decrease right slightly
//
// Only during startup.
// ============================================================

void startupBalance()
{
    float command =
        fmaxf(
            fabsf(target_left_limited),
            fabsf(target_right_limited));


    if (command < STARTUP_MIN_SPEED)
    {
        startup_active = false;
        return;
    }


    if (!startup_active)
    {
        startup_active = true;
        startup_time = millis();
    }


    if ((millis() - startup_time) >
        STARTUP_BALANCE_TIME)
    {
        startup_active = false;
        return;
    }


    float diff =
        fabsf(right_speed_control) -
        fabsf(left_speed_control);


    if (fabsf(diff) < 0.003f)
        return;


    float correction =
        diff * STARTUP_GAIN;


    correction =
        clampf(
            correction,
            -STARTUP_MAX_CORRECTION,
            STARTUP_MAX_CORRECTION);


    if (target_left_limited >= 0.0f &&
        target_right_limited >= 0.0f)
    {
        target_left_limited += correction;
        target_right_limited -= correction;
    }
    else
    {
        target_left_limited -= correction;
        target_right_limited += correction;
    }


    target_left_limited =
        clampf(
            target_left_limited,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);

    target_right_limited =
        clampf(
            target_right_limited,
            -MAX_SPEED_MS,
            MAX_SPEED_MS);
}


// ============================================================
// STEADY LEFT/RIGHT MATCH
//
// Gentle only.
// ============================================================

void steadyStateMatching()
{
    if (fabsf(target_left_limited) <
            STARTUP_MIN_SPEED ||
        fabsf(target_right_limited) <
            STARTUP_MIN_SPEED)
    {
        return;
    }


    // Do not interfere while turning.
    if (fabsf(
            target_left_limited -
            target_right_limited) > 0.02f)
    {
        return;
    }


    float diff =
        fabsf(right_speed_control) -
        fabsf(left_speed_control);


    if (fabsf(diff) <
        MATCH_DEADBAND)
    {
        return;
    }


    float correction =
        diff * MATCH_GAIN;


    correction =
        clampf(
            correction,
            -MATCH_MAX_CORRECTION,
            MATCH_MAX_CORRECTION);


    if (target_left_limited >= 0.0f &&
        target_right_limited >= 0.0f)
    {
        target_left_limited += correction;
        target_right_limited -= correction;
    }
    else
    {
        target_left_limited -= correction;
        target_right_limited += correction;
    }
}


// ============================================================
// LEFT PI
// ============================================================

float leftPI(
    float target,
    float measured,
    float dt)
{
    float target_abs =
        fabsf(target);

    float measured_abs =
        fabsf(measured);


    if (target_abs <
        STOP_CMD_THRESHOLD)
    {
        integral_left = 0.0f;
        return 0.0f;
    }


    float error =
        target_abs -
        measured_abs;


    // --------------------------------------------------------
    // Integral
    // --------------------------------------------------------

    if (fabsf(error) <
            INTEGRAL_ERROR_LIMIT &&
        measured_abs >
            INTEGRAL_MIN_SPEED)
    {
        integral_left +=
            error * dt;

        integral_left =
            clampf(
                integral_left,
                -INTEGRAL_LIMIT,
                INTEGRAL_LIMIT);
    }
    else
    {
        // Anti-windup bleed
        integral_left *= 0.95f;
    }


    float correction =
        KP_L * error +
        KI_L * integral_left;


    correction =
        clampf(
            correction,
            -PI_LIMIT,
            PI_LIMIT);


    return correction;
}


// ============================================================
// RIGHT PI
// ============================================================

float rightPI(
    float target,
    float measured,
    float dt)
{
    float target_abs =
        fabsf(target);

    float measured_abs =
        fabsf(measured);


    if (target_abs <
        STOP_CMD_THRESHOLD)
    {
        integral_right = 0.0f;
        return 0.0f;
    }


    float error =
        target_abs -
        measured_abs;


    if (fabsf(error) <
            INTEGRAL_ERROR_LIMIT &&
        measured_abs >
            INTEGRAL_MIN_SPEED)
    {
        integral_right +=
            error * dt;

        integral_right =
            clampf(
                integral_right,
                -INTEGRAL_LIMIT,
                INTEGRAL_LIMIT);
    }
    else
    {
        integral_right *= 0.95f;
    }


    float correction =
        KP_R * error +
        KI_R * integral_right;


    correction =
        clampf(
            correction,
            -PI_LIMIT,
            PI_LIMIT);


    return correction;
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void updateMotorControl(float dt)
{
    calculateWheelTarget();


    // --------------------------------------------------------
    // Target ramp
    // --------------------------------------------------------

    target_left_limited =
        rampTarget(
            target_left_limited,
            target_left,
            dt);


    target_right_limited =
        rampTarget(
            target_right_limited,
            target_right,
            dt);


    // --------------------------------------------------------
    // Startup
    // --------------------------------------------------------

    startupBalance();


    // --------------------------------------------------------
    // Steady-state matching
    // --------------------------------------------------------

    steadyStateMatching();


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (fabsf(target_left_limited) <
        STOP_CMD_THRESHOLD)
    {
        integral_left = 0.0f;

        left_dac_output =
            applySlew(
                left_dac_output,
                0.0f,
                DAC_SLEW_STOP);

        writeLeftDAC(
            left_dac_output);
    }


    if (fabsf(target_right_limited) <
        STOP_CMD_THRESHOLD)
    {
        integral_right = 0.0f;

        right_dac_output =
            applySlew(
                right_dac_output,
                0.0f,
                DAC_SLEW_STOP);

        writeRightDAC(
            right_dac_output);
    }


    // --------------------------------------------------------
    // LEFT
    // --------------------------------------------------------

    if (fabsf(target_left_limited) >=
        STARTUP_MIN_SPEED)
    {
        float ff =
            leftFeedForward(
                target_left_limited);


        float pi =
            leftPI(
                target_left_limited,
                left_speed_control,
                dt);


        float desired =
            ff + pi;


        desired =
            clampf(
                desired,
                0.0f,
                DAC_MAX);


        if (target_left_limited >= 0.0f)
            digitalWrite(L_FR, HIGH);
        else
            digitalWrite(L_FR, LOW);


        left_dac_output =
            applySlew(
                left_dac_output,
                desired,
                DAC_SLEW);


        writeLeftDAC(
            left_dac_output);
    }


    // --------------------------------------------------------
    // RIGHT
    // --------------------------------------------------------

    if (fabsf(target_right_limited) >=
        STARTUP_MIN_SPEED)
    {
        float ff =
            rightFeedForward(
                target_right_limited);


        float pi =
            rightPI(
                target_right_limited,
                right_speed_control,
                dt);


        float desired =
            ff + pi;


        desired =
            clampf(
                desired,
                0.0f,
                DAC_MAX);


        if (target_right_limited >= 0.0f)
            digitalWrite(R_FR, HIGH);
        else
            digitalWrite(R_FR, LOW);


        right_dac_output =
            applySlew(
                right_dac_output,
                desired,
                DAC_SLEW);


        writeRightDAC(
            right_dac_output);
    }
}


// ============================================================
// ODOMETRY
// ============================================================

void updateOdometry(float dt)
{
    float linear =
        (left_speed_odom +
         right_speed_odom) *
        0.5f;


    float angular =
        (right_speed_odom -
         left_speed_odom) /
        WHEEL_SEP;


    odom_x +=
        linear *
        cosf(odom_yaw) *
        dt;


    odom_y +=
        linear *
        sinf(odom_yaw) *
        dt;


    odom_yaw +=
        angular * dt;


    while (odom_yaw > PI)
        odom_yaw -=
            2.0f * PI;


    while (odom_yaw < -PI)
        odom_yaw +=
            2.0f * PI;
}


// ============================================================
// PUBLISH SPEED
// ============================================================

void publishWheelData()
{
    left_speed_msg.data =
        left_speed_control;

    right_speed_msg.data =
        right_speed_control;


    left_dac_msg.data =
        left_dac_output;

    right_dac_msg.data =
        right_dac_output;


    rcl_publish(
        &left_speed_pub,
        &left_speed_msg,
        NULL);


    rcl_publish(
        &right_speed_pub,
        &right_speed_msg,
        NULL);


    rcl_publish(
        &left_dac_pub,
        &left_dac_msg,
        NULL);


    rcl_publish(
        &right_dac_pub,
        &right_dac_msg,
        NULL);
}


// ============================================================
// PUBLISH ODOM
// ============================================================

void publishOdometry()
{
    int64_t now_ns =
        rmw_uros_epoch_nanos();


    odom_msg.header.stamp.sec =
        now_ns / 1000000000LL;

    odom_msg.header.stamp.nanosec =
        now_ns % 1000000000LL;


    odom_msg.header.frame_id.data =
        (char *)"odom";

    odom_msg.header.frame_id.size =
        4;

    odom_msg.header.frame_id.capacity =
        5;


    odom_msg.child_frame_id.data =
        (char *)"base_link";

    odom_msg.child_frame_id.size =
        9;

    odom_msg.child_frame_id.capacity =
        10;


    odom_msg.pose.pose.position.x =
        odom_x;

    odom_msg.pose.pose.position.y =
        odom_y;

    odom_msg.pose.pose.position.z =
        0.0;


    float half =
        odom_yaw * 0.5f;


    odom_msg.pose.pose.orientation.x =
        0.0;

    odom_msg.pose.pose.orientation.y =
        0.0;

    odom_msg.pose.pose.orientation.z =
        sinf(half);

    odom_msg.pose.pose.orientation.w =
        cosf(half);


    odom_msg.twist.twist.linear.x =
        (left_speed_odom +
         right_speed_odom) *
        0.5f;


    odom_msg.twist.twist.linear.y =
        0.0;


    odom_msg.twist.twist.angular.z =
        (right_speed_odom -
         left_speed_odom) /
        WHEEL_SEP;


    rcl_publish(
        &odom_pub,
        &odom_msg,
        NULL);
}


// ============================================================
// CONTROL LOOP
// ============================================================

void controlLoop()
{
    uint32_t now =
        millis();


    if ((now - last_control_ms) <
        CONTROL_PERIOD_MS)
    {
        return;
    }


    float dt =
        (now - last_control_ms) /
        1000.0f;


    last_control_ms =
        now;


    if (dt <= 0.0f ||
        dt > 0.5f)
    {
        dt =
            CONTROL_PERIOD_MS /
            1000.0f;
    }


    updateVelocity();


    updateMotorControl(dt);


    updateOdometry(dt);


    publishWheelData();


    publishOdometry();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(115200);


    // --------------------------------------------------------
    // Motor pins
    // --------------------------------------------------------

    pinMode(L_SP, OUTPUT);
    pinMode(L_FR, OUTPUT);
    pinMode(L_RUN, OUTPUT);

    pinMode(R_SP, OUTPUT);
    pinMode(R_FR, OUTPUT);
    pinMode(R_RUN, OUTPUT);


    // --------------------------------------------------------
    // Encoder pins
    // --------------------------------------------------------

    pinMode(L_HAL, INPUT_PULLUP);
    pinMode(R_HAL, INPUT_PULLUP);


    // --------------------------------------------------------
    // Motor direction
    // --------------------------------------------------------

    digitalWrite(
        L_FR,
        HIGH);

    digitalWrite(
        R_FR,
        HIGH);


    // --------------------------------------------------------
    // Start with motor OFF
    // --------------------------------------------------------

    digitalWrite(
        L_RUN,
        LOW);

    digitalWrite(
        R_RUN,
        LOW);


    writeLeftDAC(0);

    writeRightDAC(0);


    // --------------------------------------------------------
    // Encoder interrupt
    // --------------------------------------------------------

    attachInterrupt(
        digitalPinToInterrupt(L_HAL),
        leftEncoderISR,
        RISING);


    attachInterrupt(
        digitalPinToInterrupt(R_HAL),
        rightEncoderISR,
        RISING);


    // --------------------------------------------------------
    // micro-ROS
    // --------------------------------------------------------

    set_microros_serial_transports(
        Serial);


    delay(2000);


    allocator =
        rcl_get_default_allocator();


    rclc_support_init(
        &support,
        0,
        NULL,
        &allocator);


    // --------------------------------------------------------
    // NODE
    // --------------------------------------------------------

    rclc_node_init_default(
        &node,
        "wheel_speed_controller",
        "",
        &support);


    // --------------------------------------------------------
    // cmd_vel
    // --------------------------------------------------------

    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs,
            msg,
            Twist),
        "/cmd_vel");


    // --------------------------------------------------------
    // Direct left
    // --------------------------------------------------------

    rclc_subscription_init_default(
        &left_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/motor/left/cmd");


    // --------------------------------------------------------
    // Direct right
    // --------------------------------------------------------

    rclc_subscription_init_default(
        &right_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/motor/right/cmd");


    // --------------------------------------------------------
    // Reset
    // --------------------------------------------------------

    rclc_subscription_init_default(
        &pulse_reset_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Bool),
        "/motor/pulse_reset");


    // --------------------------------------------------------
    // SPEED PUB
    // --------------------------------------------------------

    rclc_publisher_init_default(
        &left_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/wheel/left/speed");


    rclc_publisher_init_default(
        &right_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/wheel/right/speed");


    // --------------------------------------------------------
    // DAC PUB
    // --------------------------------------------------------

    rclc_publisher_init_default(
        &left_dac_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/wheel/left/dac");


    rclc_publisher_init_default(
        &right_dac_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32),
        "/wheel/right/dac");


    // --------------------------------------------------------
    // ODOM
    // --------------------------------------------------------

    rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            nav_msgs,
            msg,
            Odometry),
        "/wheel/odom");


    // --------------------------------------------------------
    // Executor
    // --------------------------------------------------------

    rclc_executor_init(
        &executor,
        &support.context,
        4,
        &allocator);


    rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmdVelCallback,
        ON_NEW_DATA);


    rclc_executor_add_subscription(
        &executor,
        &left_cmd_sub,
        &left_cmd_msg,
        &leftCmdCallback,
        ON_NEW_DATA);


    rclc_executor_add_subscription(
        &executor,
        &right_cmd_sub,
        &right_cmd_msg,
        &rightCmdCallback,
        ON_NEW_DATA);


    rclc_executor_add_subscription(
        &executor,
        &pulse_reset_sub,
        &pulse_reset_msg,
        &pulseResetCallback,
        ON_NEW_DATA);


    // --------------------------------------------------------
    // Initial controller state
    // --------------------------------------------------------

    resetSpeedController();


    cmd_linear = 0.0f;
    cmd_angular = 0.0f;

    cmd_linear_filtered = 0.0f;
    cmd_angular_filtered = 0.0f;


    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_yaw = 0.0f;


    last_control_ms =
        millis();

    last_velocity_ms =
        millis();


    // --------------------------------------------------------
    // MOTOR ENABLE
    // --------------------------------------------------------

    digitalWrite(
        L_RUN,
        HIGH);

    digitalWrite(
        R_RUN,
        HIGH);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    rclc_executor_spin_some(
        &executor,
        RCL_MS_TO_NS(5));


    controlLoop();
}
