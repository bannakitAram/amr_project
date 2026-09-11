#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

// ============================================================
// STM32F407VET6 - MOTOR SPEED CONTROLLER V9.11-OPT
//
// Optimized variant of "motor_control _with_pid.cpp" (2026-09-08).
// Behavior is unchanged from that file (same gains, same calibration,
// same launch/startup-balance logic) - only dead/redundant code removed:
//
//   1) PPR interpolation removed. ppr_l_table/ppr_r_table in the
//      original are a 9-point per-speed table, but since the V9.9
//      encoder debounce fix both wheels measured the SAME constant PPR
//      (516.85, ground-truth 20-revolution manual count) at every table
//      entry - the table always returns 516.85 regardless of speed. The
//      interpolation machinery (interpolate_ppr(), get_effective_ppr_l/r(),
//      ppr_speed_table[]) was computing that same constant the hard way.
//      Replaced with a single ENCODER_PPR constant.
//
//   2) RIGHT_TRIM removed. The original divides right_dac_table by
//      RIGHT_TRIM(speed) at calibration time, then multiplies by the same
//      RIGHT_TRIM(speed) again at runtime in compute_feedforward_right() -
//      those cancel out exactly at every calibration point (verified:
//      old pretrim table * trim = new direct table, matches to within
//      rounding). Storing the trim-cancelled-out direct fit and dropping
//      the runtime multiply is mathematically identical at the 10
//      calibration points and slightly MORE accurate in between them
//      (linear interpolation of an already-linear fit, instead of
//      interpolating it and then bending it with a separate trim ramp).
//
//   3) Unused interpolate_dac() extrapolation-below-first-point parameter
//      removed. MIN_DAC_L and MIN_DAC_R were both always 0 - the
//      parameter never varied, so it's hardcoded instead of threaded
//      through both calibration tables' call sites.
//
// Everything else (Kp/Ki/Kd, DAC calibration values, slew limits, launch
// boost, startup balance, encoder debounce, CMD_TIMEOUT, etc.) is copied
// as-is from the source file - see that file's version history comments
// for the full rationale behind each of those values.
// ============================================================

// ============================================================
// PIN CONFIGURATION
// ============================================================
const int R_SP_PIN  = PA5;   // DAC / PWM Channel 2
const int R_FR_PIN  = PB0;
const int R_RUN_PIN = PC7;
const int R_HAL     = PD11;
const int L_SP_PIN  = PA4;   // DAC / PWM Channel 1
const int L_FR_PIN  = PB1;
const int L_RUN_PIN = PC6;
const int L_HAL     = PD8;

// ============================================================
// ROBOT PARAMETERS
// ============================================================
const float wheel_radius = 0.128f / 2.0f;
const float wheel_sep    = 0.472f;
const float wheel_circumference = 2.0f * PI * wheel_radius;

// ============================================================
// ENCODER PPR
// ============================================================
// Ground-truth manual count over 20 revolutions on each wheel (after
// adding the encoder debounce filter, see ENCODER_DEBOUNCE_US below) gave
// the same physical PPR for both wheels - identical motor/gearbox, so one
// constant covers both. The pre-debounce readings varied by speed and by
// wheel (619-627 left / 550-568 right) only because of noise-inflated
// pulse counts; once that noise was filtered out both wheels converged to
// the same true value.
const float ENCODER_PPR = 516.85f;

// V9.8: clamp ตามช่วงใช้งาน 0 - 1.5 m/s
const float MAX_SPEED_MS = 1.5f;

// ============================================================
// INDIVIDUAL CONTROLLER GAINS
//
// V9.10 reverted (2026-09-08): the analytical scale-down tried at V9.10
// (Kp_L 550->456, Ki_L 260->216, Kp_R 500->463, Ki_R 220->204, scaled by
// the PPR ratio) caused the controller to fail to reach target at every
// tested speed - the sensor-gain scaling theory was unverified on
// hardware and turned out to under-drive the loop, likely leaving a
// steady-state gap that the weakened integral term (and the
// fabs(measured_speed) > 0.05 anti-windup gate, at low speed) couldn't
// close in time before CMD_TIMEOUT_MS (1000 ms) zeroes the target if
// commands aren't re-sent continuously.
//
// Reverted to the original V9.7 gains, which DID reach target reliably
// (just against the wrong absolute speed value, since that was before
// the PPR fix) - use this as the empirical starting point and tune up
// or down from real step-response tests, not from further calculation.
// ============================================================
float Kp_L = 550.0f;
float Ki_L = 260.0f;
float Kd_L = 0.0f;
float Kp_R = 500.0f;
float Ki_R = 220.0f;
float Kd_R = 0.0f;

// V9.7: headroom มากพอชดเชย error ของตาราง FF ได้เต็มที่
const float INTEGRAL_LIMIT = 0.60f;
float integral_l = 0.0f;
float integral_r = 0.0f;

// ============================================================
// SPEED FILTERS (V9.7: smoother)
// ============================================================
const float CONTROL_FILTER_ALPHA_L = 0.12f;
const float CONTROL_FILTER_ALPHA_R = 0.12f;
const float ODOM_FILTER_ALPHA_L = 0.08f;
const float ODOM_FILTER_ALPHA_R = 0.08f;

// ============================================================
// CONTROL PARAMETERS
// ============================================================
const float SPEED_DEADBAND = 0.003f;   // V9.7: แน่นขึ้น
const unsigned long CONTROL_PERIOD_MS = 50;
const unsigned long VELOCITY_WINDOW_MS = 100;

// ============================================================
// ACCELERATION RAMP
// ============================================================
const float MAX_ACCEL = 0.5f;
const float MAX_DECEL = 1.2f;
const float STOP_TARGET_THRESHOLD = 0.003f;
const float STOP_SPEED_THRESHOLD = 0.035f;
float target_speed_l = 0.0f;
float target_speed_r = 0.0f;
float current_target_l = 0.0f;
float current_target_r = 0.0f;

// ============================================================
// COMMAND TIMEOUT
// ============================================================
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 1000;

// ============================================================
// DAC LIMITS
// ============================================================
const int DAC_MIN = 0;
const int DAC_MAX = 4095;

// ============================================================
// DAC SLEW LIMIT (V9.8: speed-proportional)
// ============================================================
// ปกติ slew จะแปรผันตาม target speed:
//   target ต่ำ  -> ~SLEW_BASE (นุ่ม, launch control ยังคุมได้)
//   target สูง  -> ~SLEW_MAX  (ไม่หน่วง acceleration)
const float SLEW_BASE = 35.0f;
const float SLEW_MAX  = 150.0f;
const float SLEW_FULL_SPEED = 0.8f;   // full slew ที่ target >= 0.8 m/s
const float STOP_DAC_STEP_L = 200.0f; // V9.8: เบรกทัน MAX_DECEL 1.2
const float STOP_DAC_STEP_R = 200.0f;

// ============================================================
// V9.3 LEFT LAUNCH SLEW BOOST (V9.6 re-tune)
// ============================================================
const float         LAUNCH_DAC_STEP_L           = 120.0f;
const float         LEFT_LAUNCH_SPEED_THRESHOLD = 0.005f;
const unsigned long  LEFT_LAUNCH_TIMEOUT_MS      = 700;
bool left_launch_active     = false;
bool left_launch_had_command = false;
unsigned long left_launch_start = 0;

// ============================================================
// V9.4 RIGHT LAUNCH HOLD (V9.6 re-tune)
// ============================================================
const float         RIGHT_LAUNCH_DAC_STEP   = 35.0f;
const unsigned long  RIGHT_LAUNCH_TIMEOUT_MS = 900;
bool right_launch_active      = false;
bool right_launch_had_command = false;
unsigned long right_launch_start = 0;

// ============================================================
// V9.1 / V9.6 BIDIRECTIONAL STARTUP BALANCE
// ============================================================
const unsigned long STARTUP_BALANCE_TIME_MS = 1200;
const float STARTUP_BALANCE_START_SPEED = 0.08f;
const float STARTUP_BALANCE_ERROR_ON = 0.015f;
const float STARTUP_BALANCE_ERROR_OFF = 0.006f;
const float STARTUP_BALANCE_ALPHA = 0.35f;
const float STARTUP_BALANCE_K = 180.0f;
const float STARTUP_BALANCE_MAX_DAC = 60.0f;   // V9.8: แรงขึ้นเพื่อปิด gap หลัง launch
float startup_speed_diff = 0.0f;
float startup_left_correction = 0.0f;
float startup_right_correction = 0.0f;
unsigned long startup_balance_start = 0;
bool startup_balance_active = false;
bool startup_balance_had_command = false;

// ============================================================
// INDIVIDUAL WHEEL DAC CALIBRATION TABLE
//
// V9.11 recalibration (2026-09-08): re-collected wheel_data010/020/030
// CLOSED-LOOP (current firmware, corrected PPR, reverted Kp/Ki) and used
// the fully-converged steady-state (dac, speed) pairs directly as ground
// truth - std dev on both dac and speed was negligible in the last ~10%
// of each run, i.e. true equilibrium. (wheel_data040 was excluded: at
// 30 s it had not yet converged - L was still 0.383 vs target 0.40 - so
// its dac/speed pair does not represent equilibrium and would bias the
// fit.)
//   left  total_dac(speed) = 3257.14 * speed + 57.34   (resid <= 1.2 counts)
//   right total_dac(speed) = 3250.05 * speed + 55.16   (resid <= 0.1 counts)
// Both slopes land close together (~3250), consistent with identical
// motor/gearbox hardware on both wheels.
//
// Both tables store total_dac(speed) directly - no per-wheel trim step
// (see file header: the original RIGHT_TRIM cancelled itself out exactly
// at every calibration point, so it was removed here rather than kept as
// a no-op multiply).
//
// CAUTION: only 0.10-0.30 m/s were fit from real converged data; 0.40-1.00
// m/s remain a LINEAR EXTRAPOLATION beyond the measured range (0.40 is
// close to measured but not verified converged) - re-collect wheel_data at
// 0.40-0.50 m/s and let it run long enough to fully converge (>15 s, watch
// dac AND speed both go flat) before trusting that region in production.
// ============================================================
const int CAL_POINTS = 10;
const float speed_table[CAL_POINTS] =
{
  0.10f, 0.20f, 0.30f, 0.40f, 0.50f,
  0.60f, 0.70f, 0.80f, 0.90f, 1.00f
};

const float left_dac_table[CAL_POINTS] =
{
  383.05f, 708.76f, 1034.48f, 1360.19f, 1685.91f,
  2011.62f, 2337.33f, 2663.05f, 2988.76f, 3314.47f
};

const float right_dac_table[CAL_POINTS] =
{
  380.17f, 705.17f, 1030.18f, 1355.18f, 1680.19f,
  2005.19f, 2330.20f, 2655.20f, 2980.21f, 3305.21f
};

// ============================================================
// ENCODER
// ============================================================
volatile long pulse_l = 0;
volatile long pulse_r = 0;
volatile long cal_pulse_l = 0;
volatile long cal_pulse_r = 0;

long velocity_pulse_l = 0;
long velocity_pulse_r = 0;

// V9.7: 6 samples -> span ~250 ms, raw เนียนขึ้น (±1 pulse ~ ±0.003)
const int VELOCITY_HISTORY_SIZE = 6;
long pulse_history_l[VELOCITY_HISTORY_SIZE] = {0};
long pulse_history_r[VELOCITY_HISTORY_SIZE] = {0};
unsigned long time_history[VELOCITY_HISTORY_SIZE] = {0};
int history_index = 0;
bool history_initialized = false;

// ============================================================
// SPEED VARIABLES
// ============================================================
float raw_speed_l = 0.0f;
float raw_speed_r = 0.0f;
float control_speed_l = 0.0f;
float control_speed_r = 0.0f;
float odom_speed_l = 0.0f;
float odom_speed_r = 0.0f;

// ============================================================
// PID STATE
// ============================================================
float prev_error_l = 0.0f;
float prev_error_r = 0.0f;

// ============================================================
// DAC STATE
// ============================================================
float desired_dac_l = 0.0f;
float desired_dac_r = 0.0f;
int dac_output_l = 0;
int dac_output_r = 0;

// ============================================================
// ODOMETRY
// ============================================================
float pose_x = 0.0f;
float pose_y = 0.0f;
float pose_theta = 0.0f;
unsigned long last_control_time = 0;
unsigned long last_pulse_pub_time = 0;

// ============================================================
// MICRO-ROS
// ============================================================
enum States
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
States state;

// ============================================================
// ROS OBJECTS
// ============================================================
rcl_subscription_t subscriber;
rcl_subscription_t sub_left_cmd;
rcl_subscription_t sub_right_cmd;
rcl_subscription_t sub_pulse_reset;
rcl_publisher_t odom_publisher;
rcl_publisher_t left_speed_publisher;
rcl_publisher_t right_speed_publisher;
rcl_publisher_t left_dac_publisher;
rcl_publisher_t right_dac_publisher;
rcl_publisher_t left_pulse_publisher;
rcl_publisher_t right_pulse_publisher;

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Float32 msg_left_cmd;
std_msgs__msg__Float32 msg_right_cmd;
std_msgs__msg__Bool msg_pulse_reset;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float32 left_speed_msg;
std_msgs__msg__Float32 right_speed_msg;
std_msgs__msg__Float32 left_dac_msg;
std_msgs__msg__Float32 right_dac_msg;
std_msgs__msg__Int32 left_pulse_msg;
std_msgs__msg__Int32 right_pulse_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
unsigned long last_ping_time = 0;

// ============================================================
// V9.5 TIME SYNC STATE
// ============================================================
unsigned long connected_at_ms = 0;
bool time_resynced = false;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
bool create_entities();
void destroy_entities();
void reset_speed_controller();
void check_cmd_timeout();
void update_ramped_targets(float dt);
void read_encoder_counts(long &count_l, long &count_r);
void update_velocity_estimator(long count_l, long count_r, unsigned long now);
void update_control_filters();
void update_odom_filters();
float interpolate_dac(float target_speed, const float * dac_table);
float compute_feedforward_left(float target_speed);
float compute_feedforward_right(float target_speed);
float compute_left_dac(float target_speed, float measured_speed, float dt);
float compute_right_dac(float target_speed, float measured_speed, float dt);
int apply_dac_slew(float desired, int current, float max_step);
void update_left_launch();
void update_right_launch();
void update_startup_balance();
float get_startup_left_correction();
float get_startup_right_correction();
void motor_control_process();
void update_and_publish_odom();
void publish_pulse_counts();
void L_HAL_ISR();
void R_HAL_ISR();

// ============================================================
// ENCODER ISR
// ============================================================
// Software debounce (เทียบเท่า Timer Input Filter สำหรับ EXTI ที่ไม่มี ICxF):
// ปฏิเสธ edge ที่มาเร็วเกินไปจาก edge ก่อนหน้า - ที่ 1.5 m/s (max speed) กับ
// PPR ~627 (left) period สั้นสุดของ pulse จริงอยู่ที่ ~427 us จึงตั้ง window
// ไว้ 50 us ให้กัน contact bounce/noise ได้โดยไม่กิน pulse จริงแม้ที่ท็อปสปีด
volatile unsigned long last_pulse_us_l = 0;
volatile unsigned long last_pulse_us_r = 0;
const unsigned long ENCODER_DEBOUNCE_US = 50;

void L_HAL_ISR()
{
  unsigned long now = micros();
  if (now - last_pulse_us_l < ENCODER_DEBOUNCE_US) return;
  last_pulse_us_l = now;
  pulse_l++; cal_pulse_l++;
}

void R_HAL_ISR()
{
  unsigned long now = micros();
  if (now - last_pulse_us_r < ENCODER_DEBOUNCE_US) return;
  last_pulse_us_r = now;
  pulse_r++; cal_pulse_r++;
}

// ============================================================
// CMD_VEL CALLBACK
// ============================================================
void subscription_callback(const void * msvin)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msvin;
  float v = msg->linear.x;
  float w = msg->angular.z;

  target_speed_l = v - ((w * wheel_sep) / 2.0f);
  target_speed_r = v + ((w * wheel_sep) / 2.0f);

  target_speed_l = constrain(target_speed_l, -MAX_SPEED_MS, MAX_SPEED_MS);
  target_speed_r = constrain(target_speed_r, -MAX_SPEED_MS, MAX_SPEED_MS);
  last_cmd_time = millis();
}

// ============================================================
// DIRECT LEFT COMMAND
// ============================================================
void left_cmd_callback(const void * msvin)
{
  const std_msgs__msg__Float32 * msg =
    (const std_msgs__msg__Float32 *)msvin;
  target_speed_l = constrain(msg->data, -MAX_SPEED_MS, MAX_SPEED_MS);
  last_cmd_time = millis();
}

// ============================================================
// DIRECT RIGHT COMMAND
// ============================================================
void right_cmd_callback(const void * msvin)
{
  const std_msgs__msg__Float32 * msg =
    (const std_msgs__msg__Float32 *)msvin;
  target_speed_r = constrain(msg->data, -MAX_SPEED_MS, MAX_SPEED_MS);
  last_cmd_time = millis();
}

// ============================================================
// RESET CALIBRATION COUNTERS
// ============================================================
void pulse_reset_callback(const void * msvin)
{
  const std_msgs__msg__Bool * msg =
    (const std_msgs__msg__Bool *)msvin;
  if (msg->data)
  {
    noInterrupts();
    cal_pulse_l = 0;
    cal_pulse_r = 0;
    interrupts();
  }
}

// ============================================================
// MICRO-ROS CREATE ENTITIES
// ============================================================
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "stm32_motor_node", "", &support) != RCL_RET_OK) return false;

  rmw_uros_sync_session(1000);

  // SUBSCRIBERS
  if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_left_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor/left/cmd") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_right_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor/right/cmd") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_pulse_reset, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "motor/pulse_reset") != RCL_RET_OK) return false;

  // PUBLISHERS
  if (rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel/odom") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&left_speed_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel/left/speed") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&right_speed_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel/right/speed") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&left_dac_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel/left/dac") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&right_dac_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel/right/dac") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&left_pulse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "wheel/left/pulse_count") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&right_pulse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "wheel/right/pulse_count") != RCL_RET_OK) return false;

  // EXECUTOR
  if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_left_cmd, &msg_left_cmd, left_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_right_cmd, &msg_right_cmd, right_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_pulse_reset, &msg_pulse_reset, pulse_reset_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // ODOM FRAME NAMES
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  reset_speed_controller();
  last_control_time = millis();
  return true;
}

// ============================================================
// DESTROY ENTITIES
// ============================================================
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rosidl_runtime_c__String__fini(&odom_msg.header.frame_id);
  rosidl_runtime_c__String__fini(&odom_msg.child_frame_id);

  (void) rcl_publisher_fini(&odom_publisher, &node);
  (void) rcl_publisher_fini(&left_speed_publisher, &node);
  (void) rcl_publisher_fini(&right_speed_publisher, &node);
  (void) rcl_publisher_fini(&left_dac_publisher, &node);
  (void) rcl_publisher_fini(&right_dac_publisher, &node);
  (void) rcl_publisher_fini(&left_pulse_publisher, &node);
  (void) rcl_publisher_fini(&right_pulse_publisher, &node);

  (void) rcl_subscription_fini(&subscriber, &node);
  (void) rcl_subscription_fini(&sub_left_cmd, &node);
  (void) rcl_subscription_fini(&sub_right_cmd, &node);
  (void) rcl_subscription_fini(&sub_pulse_reset, &node);

  (void) rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  (void) rclc_support_fini(&support);
}

// ============================================================
// RESET SPEED CONTROLLER
// ============================================================
void reset_speed_controller()
{
  raw_speed_l = 0.0f; raw_speed_r = 0.0f;
  control_speed_l = 0.0f; control_speed_r = 0.0f;
  odom_speed_l = 0.0f; odom_speed_r = 0.0f;
  prev_error_l = 0.0f; prev_error_r = 0.0f;
  integral_l = 0.0f; integral_r = 0.0f;
  desired_dac_l = 0.0f; desired_dac_r = 0.0f;
  dac_output_l = 0; dac_output_r = 0;
  left_dac_msg.data = 0.0f; right_dac_msg.data = 0.0f;
  current_target_l = 0.0f; current_target_r = 0.0f;

  startup_speed_diff = 0.0f;
  startup_left_correction = 0.0f;
  startup_right_correction = 0.0f;
  startup_balance_start = 0;
  startup_balance_active = false;
  startup_balance_had_command = false;

  left_launch_active = false;
  left_launch_had_command = false;
  left_launch_start = 0;

  right_launch_active = false;
  right_launch_had_command = false;
  right_launch_start = 0;

  velocity_pulse_l = 0; velocity_pulse_r = 0;
  history_index = 0; history_initialized = false;
  for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++)
  {
    pulse_history_l[i] = 0; pulse_history_r[i] = 0; time_history[i] = 0;
  }
}

// ============================================================
// COMMAND TIMEOUT
// ============================================================
void check_cmd_timeout()
{
  if (last_cmd_time == 0 || millis() - last_cmd_time > CMD_TIMEOUT_MS)
  {
    target_speed_l = 0.0f;
    target_speed_r = 0.0f;
  }
}

// ============================================================
// ACCELERATION RAMP
// ============================================================
void update_ramped_targets(float dt)
{
  float accel_step = MAX_ACCEL * dt;
  float decel_step = MAX_DECEL * dt;

  if (fabs(target_speed_l) < STOP_TARGET_THRESHOLD) current_target_l = 0.0f;
  else if (current_target_l < target_speed_l) current_target_l = min(current_target_l + accel_step, target_speed_l);
  else if (current_target_l > target_speed_l) current_target_l = max(current_target_l - decel_step, target_speed_l);

  if (fabs(target_speed_r) < STOP_TARGET_THRESHOLD) current_target_r = 0.0f;
  else if (current_target_r < target_speed_r) current_target_r = min(current_target_r + accel_step, target_speed_r);
  else if (current_target_r > target_speed_r) current_target_r = max(current_target_r - decel_step, target_speed_r);
}

// ============================================================
// READ ENCODER
// ============================================================
void read_encoder_counts(long &count_l, long &count_r)
{
  noInterrupts();
  count_l = pulse_l; count_r = pulse_r;
  pulse_l = 0; pulse_r = 0;
  interrupts();
}

// ============================================================
// VELOCITY ESTIMATOR
// ============================================================
void update_velocity_estimator(long count_l, long count_r, unsigned long now)
{
  bool left_forward = current_target_l >= 0.0f;
  bool right_forward = current_target_r >= 0.0f;

  long signed_l = left_forward ? count_l : -count_l;
  long signed_r = right_forward ? count_r : -count_r;

  velocity_pulse_l += signed_l;
  velocity_pulse_r += signed_r;

  if (!history_initialized)
  {
    for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++)
    {
      pulse_history_l[i] = velocity_pulse_l;
      pulse_history_r[i] = velocity_pulse_r;
      time_history[i] = now;
    }
    history_index = 0;
    history_initialized = true;
    raw_speed_l = 0.0f;
    raw_speed_r = 0.0f;
    return;
  }

  pulse_history_l[history_index] = velocity_pulse_l;
  pulse_history_r[history_index] = velocity_pulse_r;
  time_history[history_index] = now;

  int oldest_index = (history_index + 1) % VELOCITY_HISTORY_SIZE;
  unsigned long elapsed_ms = now - time_history[oldest_index];

  if (elapsed_ms >= VELOCITY_WINDOW_MS)
  {
    long delta_pulse_l = velocity_pulse_l - pulse_history_l[oldest_index];
    long delta_pulse_r = velocity_pulse_r - pulse_history_r[oldest_index];

    float distance_l = ((float)delta_pulse_l / ENCODER_PPR) * wheel_circumference;
    float distance_r = ((float)delta_pulse_r / ENCODER_PPR) * wheel_circumference;

    float dt = elapsed_ms / 1000.0f;
    if (dt > 0.0f)
    {
      raw_speed_l = distance_l / dt;
      raw_speed_r = distance_r / dt;
    }
  }
  history_index = (history_index + 1) % VELOCITY_HISTORY_SIZE;
}

// ============================================================
// LOW PASS FILTERS
// ============================================================
void update_control_filters()
{
  control_speed_l = CONTROL_FILTER_ALPHA_L * raw_speed_l + (1.0f - CONTROL_FILTER_ALPHA_L) * control_speed_l;
  control_speed_r = CONTROL_FILTER_ALPHA_R * raw_speed_r + (1.0f - CONTROL_FILTER_ALPHA_R) * control_speed_r;
}

void update_odom_filters()
{
  odom_speed_l = ODOM_FILTER_ALPHA_L * raw_speed_l + (1.0f - ODOM_FILTER_ALPHA_L) * odom_speed_l;
  odom_speed_r = ODOM_FILTER_ALPHA_R * raw_speed_r + (1.0f - ODOM_FILTER_ALPHA_R) * odom_speed_r;
}

// ============================================================
// DAC INTERPOLATION (extrapolates below speed_table[0] from the origin,
// and beyond speed_table[CAL_POINTS-1] linearly - see calibration comment)
// ============================================================
float interpolate_dac(float target_speed, const float * dac_table)
{
  float speed = fabs(target_speed);
  if (speed <= 0.001f) return 0.0f;

  if (speed <= speed_table[0])
  {
    float ratio = speed / speed_table[0];
    return constrain(ratio * dac_table[0], (float)DAC_MIN, (float)DAC_MAX);
  }

  for (int i = 0; i < CAL_POINTS - 1; i++)
  {
    if (speed >= speed_table[i] && speed <= speed_table[i + 1])
    {
      float ratio = (speed - speed_table[i]) / (speed_table[i + 1] - speed_table[i]);
      float result = dac_table[i] + ratio * (dac_table[i + 1] - dac_table[i]);
      return constrain(result, (float)DAC_MIN, (float)DAC_MAX);
    }
  }

  // V9.8 note: > 1.0 m/s ใช้ extrapolation แล้ว constrain ที่ 4095
  // (ไม่ถึง 1.5 ก็ไม่เป็นไร - DAC จะอิ่มที่เพดาน)
  float slope = (dac_table[CAL_POINTS - 1] - dac_table[CAL_POINTS - 2]) / (speed_table[CAL_POINTS - 1] - speed_table[CAL_POINTS - 2]);
  float result = dac_table[CAL_POINTS - 1] + slope * (speed - speed_table[CAL_POINTS - 1]);
  return constrain(result, (float)DAC_MIN, (float)DAC_MAX);
}

// ============================================================
// FEED-FORWARD
// ============================================================
float compute_feedforward_left(float target_speed)
{
  return interpolate_dac(target_speed, left_dac_table);
}

float compute_feedforward_right(float target_speed)
{
  return interpolate_dac(target_speed, right_dac_table);
}

// ============================================================
// V9.3 LEFT LAUNCH BOOST STATE
// ============================================================
void update_left_launch()
{
  bool commanded = fabs(current_target_l) > 0.001f;
  if (!commanded)
  {
    left_launch_active = false;
    left_launch_had_command = false;
    return;
  }
  if (!left_launch_had_command)
  {
    left_launch_had_command = true;
    left_launch_active = true;
    left_launch_start = millis();
  }
  if (left_launch_active)
  {
    bool moving = fabs(raw_speed_l) > LEFT_LAUNCH_SPEED_THRESHOLD;
    bool timed_out = (millis() - left_launch_start) > LEFT_LAUNCH_TIMEOUT_MS;
    if (moving || timed_out) left_launch_active = false;
  }
}

// ============================================================
// V9.4 RIGHT LAUNCH HOLD STATE
// ============================================================
void update_right_launch()
{
  bool commanded = fabs(current_target_r) > 0.001f;
  if (!commanded)
  {
    right_launch_active = false;
    right_launch_had_command = false;
    return;
  }
  if (!right_launch_had_command)
  {
    right_launch_had_command = true;
    right_launch_active = true;
    right_launch_start = millis();
  }
  if (right_launch_active)
  {
    bool left_moving = fabs(raw_speed_l) > LEFT_LAUNCH_SPEED_THRESHOLD;
    bool timed_out = (millis() - right_launch_start) > RIGHT_LAUNCH_TIMEOUT_MS;
    if (left_moving || timed_out) right_launch_active = false;
  }
}

// ============================================================
// V9.1 / V9.6 BIDIRECTIONAL STARTUP BALANCE
// ============================================================
void update_startup_balance()
{
  bool commanded = fabs(current_target_l) > STARTUP_BALANCE_START_SPEED ||
                   fabs(current_target_r) > STARTUP_BALANCE_START_SPEED;

  if (!commanded)
  {
    startup_balance_active = false;
    startup_balance_had_command = false;
    startup_speed_diff = 0.0f;
    startup_left_correction = 0.0f;
    startup_right_correction = 0.0f;
    return;
  }

  if (!startup_balance_had_command)
  {
    startup_balance_had_command = true;
    startup_balance_active = true;
    startup_balance_start = millis();
    startup_speed_diff = 0.0f;
    startup_left_correction = 0.0f;
    startup_right_correction = 0.0f;
  }

  if (!startup_balance_active)
  {
    startup_left_correction = 0.0f;
    startup_right_correction = 0.0f;
    return;
  }

  if (millis() - startup_balance_start > STARTUP_BALANCE_TIME_MS)
  {
    startup_balance_active = false;
    startup_left_correction = 0.0f;
    startup_right_correction = 0.0f;
    return;
  }

  float diff = fabs(raw_speed_r) - fabs(raw_speed_l);
  startup_speed_diff = STARTUP_BALANCE_ALPHA * diff + (1.0f - STARTUP_BALANCE_ALPHA) * startup_speed_diff;

  // V9.6: bidirectional -- ช่วยข้างที่ "ช้ากว่า"
  if (startup_speed_diff > STARTUP_BALANCE_ERROR_ON)
  {
    startup_left_correction  = constrain(STARTUP_BALANCE_K * (startup_speed_diff - STARTUP_BALANCE_ERROR_OFF), 0.0f, STARTUP_BALANCE_MAX_DAC);
    startup_right_correction = 0.0f;
  }
  else if (startup_speed_diff < -STARTUP_BALANCE_ERROR_ON)
  {
    startup_right_correction = constrain(STARTUP_BALANCE_K * (-startup_speed_diff - STARTUP_BALANCE_ERROR_OFF), 0.0f, STARTUP_BALANCE_MAX_DAC);
    startup_left_correction  = 0.0f;
  }
  else
  {
    startup_left_correction  = 0.0f;
    startup_right_correction = 0.0f;
  }
}

float get_startup_left_correction()  { return startup_left_correction; }
float get_startup_right_correction() { return startup_right_correction; }

// ============================================================
// CONTROLLERS (V9.7/V9.8 anti-windup)
// ============================================================
float compute_left_dac(float target_speed, float measured_speed, float dt)
{
  float target_mag = fabs(target_speed);
  if (target_mag <= 0.001f)
  {
    prev_error_l = 0.0f; integral_l = 0.0f; return 0.0f;
  }

  float error = target_mag - fabs(measured_speed);
  if (fabs(error) < SPEED_DEADBAND) error = 0.0f;

  float derivative = (dt > 0.0f) ? (error - prev_error_l) / dt : 0.0f;

  // V9.7: ไม่ integrate ตอนล้อหยุด (กัน wind-up ตอนออกตัว)
  // V9.8: ไม่ integrate ขึ้นต่อตอน DAC อิ่มตัว (กัน wind-up ที่ target สูงเกินเอื้อม)
  bool saturated = (desired_dac_l >= (float)(DAC_MAX - 1)) && (error > 0.0f);
  if (dt > 0.0f && fabs(error) < 0.20f &&
      fabs(measured_speed) > 0.05f && !saturated)
  {
    integral_l += error * dt;
    integral_l = constrain(integral_l, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  prev_error_l = error;

  float ff = compute_feedforward_left(target_speed);
  float correction = Kp_L * error + Ki_L * integral_l + Kd_L * derivative;
  float output = ff + correction + get_startup_left_correction();

  return constrain(output, (float)DAC_MIN, (float)DAC_MAX);
}

float compute_right_dac(float target_speed, float measured_speed, float dt)
{
  float target_mag = fabs(target_speed);
  if (target_mag <= 0.001f)
  {
    prev_error_r = 0.0f; integral_r = 0.0f; return 0.0f;
  }

  float error = target_mag - fabs(measured_speed);
  if (fabs(error) < SPEED_DEADBAND) error = 0.0f;

  float derivative = (dt > 0.0f) ? (error - prev_error_r) / dt : 0.0f;

  bool saturated = (desired_dac_r >= (float)(DAC_MAX - 1)) && (error > 0.0f);
  if (dt > 0.0f && fabs(error) < 0.20f &&
      fabs(measured_speed) > 0.05f && !saturated)
  {
    integral_r += error * dt;
    integral_r = constrain(integral_r, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  prev_error_r = error;

  float ff = compute_feedforward_right(target_speed);
  float correction = Kp_R * error + Ki_R * integral_r + Kd_R * derivative;
  float output = ff + correction + get_startup_right_correction();

  return constrain(output, (float)DAC_MIN, (float)DAC_MAX);
}

// ============================================================
// DAC SLEW LIMITER
// ============================================================
int apply_dac_slew(float desired, int current, float max_step)
{
  float difference = desired - (float)current;
  if (difference > max_step) difference = max_step;
  if (difference < -max_step) difference = -max_step;
  return constrain((int)((float)current + difference), DAC_MIN, DAC_MAX);
}

// ============================================================
// MOTOR OUTPUT
// ============================================================
void motor_control_process()
{
  digitalWrite(L_FR_PIN, (current_target_l >= 0.0f) ? HIGH : LOW);
  digitalWrite(R_FR_PIN, (current_target_r >= 0.0f) ? LOW : HIGH);
  analogWrite(L_SP_PIN, dac_output_l);
  analogWrite(R_SP_PIN, dac_output_r);
}

// ============================================================
// UPDATE CONTROL + ODOM
// ============================================================
void update_and_publish_odom()
{
  unsigned long now = millis();
  unsigned long elapsed = now - last_control_time;
  if (elapsed < CONTROL_PERIOD_MS) return;

  float dt = elapsed / 1000.0f;
  if (dt <= 0.0f) return;
  if (dt > 0.2f) dt = 0.05f;
  last_control_time = now;

  update_ramped_targets(dt);

  long count_l = 0, count_r = 0;
  read_encoder_counts(count_l, count_r);
  update_velocity_estimator(count_l, count_r, now);

  update_control_filters();
  update_odom_filters();

  update_startup_balance();
  update_left_launch();
  update_right_launch();

  if (fabs(target_speed_l) < STOP_TARGET_THRESHOLD && fabs(target_speed_r) < STOP_TARGET_THRESHOLD)
  {
    if (fabs(raw_speed_l) < STOP_SPEED_THRESHOLD) odom_speed_l = 0.0f;
    if (fabs(raw_speed_r) < STOP_SPEED_THRESHOLD) odom_speed_r = 0.0f;
  }

  desired_dac_l = compute_left_dac(current_target_l, control_speed_l, dt);
  desired_dac_r = compute_right_dac(current_target_r, control_speed_r, dt);

  // V9.8: speed-proportional normal slew
  float slew_l = SLEW_BASE + (SLEW_MAX - SLEW_BASE) * constrain(fabs(current_target_l) / SLEW_FULL_SPEED, 0.0f, 1.0f);
  float slew_r = SLEW_BASE + (SLEW_MAX - SLEW_BASE) * constrain(fabs(current_target_r) / SLEW_FULL_SPEED, 0.0f, 1.0f);

  // LEFT SLEW
  float dac_step_l = (fabs(target_speed_l) < STOP_TARGET_THRESHOLD) ? STOP_DAC_STEP_L :
                     (left_launch_active) ? LAUNCH_DAC_STEP_L : slew_l;
  dac_output_l = apply_dac_slew(desired_dac_l, dac_output_l, dac_step_l);

  // RIGHT SLEW
  float dac_step_r = (fabs(target_speed_r) < STOP_TARGET_THRESHOLD) ? STOP_DAC_STEP_R :
                     (right_launch_active) ? RIGHT_LAUNCH_DAC_STEP : slew_r;
  dac_output_r = apply_dac_slew(desired_dac_r, dac_output_r, dac_step_r);

  // PUBLISH SPEED
  left_speed_msg.data = odom_speed_l;
  rcl_publish(&left_speed_publisher, &left_speed_msg, NULL);
  right_speed_msg.data = odom_speed_r;
  rcl_publish(&right_speed_publisher, &right_speed_msg, NULL);

  // PUBLISH DAC
  left_dac_msg.data = (float)dac_output_l;
  rcl_publish(&left_dac_publisher, &left_dac_msg, NULL);
  right_dac_msg.data = (float)dac_output_r;
  rcl_publish(&right_dac_publisher, &right_dac_msg, NULL);

  // ODOMETRY CALCULATION
  float v = (odom_speed_r + odom_speed_l) / 2.0f;
  float w = (odom_speed_r - odom_speed_l) / wheel_sep;
  float delta_s = v * dt;
  float delta_theta = w * dt;

  pose_x += delta_s * cos(pose_theta + delta_theta / 2.0f);
  pose_y += delta_s * sin(pose_theta + delta_theta / 2.0f);
  pose_theta += delta_theta;

  while (pose_theta > PI) pose_theta -= 2.0f * PI;
  while (pose_theta < -PI) pose_theta += 2.0f * PI;

  // ROS TIME
  int64_t time_ms = rmw_uros_epoch_millis();
  odom_msg.header.stamp.sec = time_ms / 1000;
  odom_msg.header.stamp.nanosec = (time_ms % 1000) * 1000000;

  // POSITION
  odom_msg.pose.pose.position.x = pose_x;
  odom_msg.pose.pose.position.y = pose_y;
  odom_msg.pose.pose.position.z = 0.0;

  // ORIENTATION
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(pose_theta / 2.0f);
  odom_msg.pose.pose.orientation.w = cos(pose_theta / 2.0f);

  // VELOCITY
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

// ============================================================
// PUBLISH PULSE COUNTERS
// ============================================================
void publish_pulse_counts()
{
  unsigned long now = millis();
  if (now - last_pulse_pub_time < 200) return;

  long l = 0, r = 0;
  noInterrupts();
  l = cal_pulse_l; r = cal_pulse_r;
  interrupts();

  left_pulse_msg.data = (int32_t)l;
  rcl_publish(&left_pulse_publisher, &left_pulse_msg, NULL);
  right_pulse_msg.data = (int32_t)r;
  rcl_publish(&right_pulse_publisher, &right_pulse_msg, NULL);
  last_pulse_pub_time = now;
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
  pinMode(R_FR_PIN, OUTPUT); pinMode(R_RUN_PIN, OUTPUT);
  pinMode(L_FR_PIN, OUTPUT); pinMode(L_RUN_PIN, OUTPUT);
  pinMode(L_HAL, INPUT_PULLUP); pinMode(R_HAL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_HAL), L_HAL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_HAL), R_HAL_ISR, CHANGE);

  analogWriteResolution(12);
  analogWrite(R_SP_PIN, 0); analogWrite(L_SP_PIN, 0);

  digitalWrite(R_FR_PIN, LOW); digitalWrite(L_FR_PIN, LOW);

  digitalWrite(R_RUN_PIN, HIGH); digitalWrite(L_RUN_PIN, HIGH);
  delay(500);
  digitalWrite(R_RUN_PIN, LOW); digitalWrite(L_RUN_PIN, LOW);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  state = WAITING_AGENT;
  last_control_time = millis();
  last_cmd_time = 0;
  reset_speed_controller();
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
  switch (state)
  {
    case WAITING_AGENT:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == AGENT_CONNECTED)
      {
        connected_at_ms = millis();
        time_resynced = false;
      }
      else
      {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      if (millis() - last_ping_time > 2000)
      {
        last_ping_time = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
        {
          state = AGENT_DISCONNECTED;
          break;
        }
      }

      // V9.5: re-sync เวลาหลัง connect นิ่งแล้ว 3 วิ
      if (!time_resynced && (millis() - connected_at_ms) > 3000)
      {
        rmw_uros_sync_session(1000);
        time_resynced = true;
      }

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      update_and_publish_odom();
      publish_pulse_counts();
      break;

    case AGENT_DISCONNECTED:
      target_speed_l = 0.0f; target_speed_r = 0.0f;
      reset_speed_controller();
      dac_output_l = 0; dac_output_r = 0;
      motor_control_process();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  if (state == AGENT_CONNECTED) check_cmd_timeout();
  else { target_speed_l = 0.0f; target_speed_r = 0.0f; }

  motor_control_process();
}
