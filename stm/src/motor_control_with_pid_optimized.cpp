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
#include <sensor_msgs/msg/imu.h>
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
// Per-wheel effective radius (m). Both default to the nominal 0.128 m
// diameter, so behaviour is unchanged until calibrated: a common scale
// error is fixed by moving both together, a left/right ratio (straight-line
// drift) by moving them apart - see calib_straight_node.py. Use the loaded,
// on-floor effective radius (tyre sag under payload shrinks it), not the
// unloaded catalogue value. Do NOT compensate via ENCODER_PPR (ground truth).
const float wheel_radius_l = 0.128f / 2.0f;
const float wheel_radius_r = 0.128f / 2.0f;
const float wheel_sep    = 0.472f;
const float wheel_circumference_l = 2.0f * PI * wheel_radius_l;
const float wheel_circumference_r = 2.0f * PI * wheel_radius_r;

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
// Window for the ODOMETRY speed only (pose + /wheel/odom twist + slip check).
// The PID path (raw_speed_l/r) still uses the oldest history slot = 5 ticks
// = ~250 ms (the tuned V9.x behaviour) - do NOT shorten that without
// re-tuning the PID. The odom path instead picks the newest history slot
// that is at least ODOM_WINDOW_MS old (150 ms at the 50 ms tick, i.e.
// ~75 ms mean delay instead of ~125 ms) and needs no EMA on top.
const unsigned long ODOM_WINDOW_MS = 150;

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
// Lowered from 1000ms (2026-09-17): at 1000ms a stalled/blocked high-level
// control loop (e.g. GIL contention on the Jetson side) could keep the
// robot moving at the last received cmd_vel for up to a full second after
// the caller intended to stop, causing measured overshoot on "move forward
// X meters" tests (~0.15-0.2m at 0.15 m/s). 200ms still gives >2x margin
// over typical cmd_vel publish periods (teleop/joystick ~33-100ms, Nav2
// controller ~50ms) while bounding failsafe-stop latency much tighter.
const unsigned long CMD_TIMEOUT_MS = 200;

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
const float         LAUNCH_DAC_STEP_L           = 35.0f;
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
// WHEEL DIRECTION LATCH (2026-10-16)
// ============================================================
// The Hall encoders are single-channel (no direction), and both the
// encoder sign and the FR (direction) pin used to follow the sign of
// current_target with 0 counted as FORWARD. Stopping from REVERSE therefore
// flipped FR to forward while the DAC was still slewing down and the wheel
// still rolling backwards: the driver drove against the motion (plug
// braking -> visible wheel slip, measured wheel decel ~0.6 vs body ~0.25
// m/s^2) and the remaining backward pulses were counted as forward
// (fake +0.05..0.07 m/s "overshoot", wrong odom). Forward stops were fine
// because 0 already meant forward.
// Now each wheel keeps its last direction until its DAC has slewed to 0
// AND the wheel has (nearly) stopped, or DIR_CHANGE_TIMEOUT_MS after the
// DAC reached 0; while a new command disagrees with the latched direction
// the DAC is driven to 0 first.
const unsigned long DIR_CHANGE_TIMEOUT_MS = 300;
int dir_l = 1;
int dir_r = 1;
unsigned long dac_zero_since_l = 0;
unsigned long dac_zero_since_r = 0;

// ============================================================
// INDIVIDUAL WHEEL DAC CALIBRATION TABLE
//
// V9.15 shared-table experiment (2026-09-11): right_dac_table forced
// IDENTICAL to left_dac_table (both use the V9.11 left fit) at the
// user's request, instead of a separately measured right-side table.
//
// WARNING - this contradicts every wheel_data measurement taken so far.
// Every closed-loop capture (V9.12 through V9.14, wheel_data010-100)
// showed the right wheel consistently needs MORE dac than the left to
// reach the same speed (e.g. 467 vs 386 counts at 0.10 m/s, ~495-1954
// vs ~386-1694 across the range) - not a calibration artifact, since it
// held up across 10 independently re-collected speed points. Forcing
// the tables equal means the right-side FF will under-drive at every
// speed; the PID integral (Ki_R, limited to +-INTEGRAL_LIMIT) is what
// will have to make up the gap in real time, which is exactly the
// failure mode already observed at 0.40 m/s in the original V9.11 data
// (integral saturated, 2.7% steady-state speed error, right wheel could
// not reach target). Expect the same or worse at higher speeds where
// the FF gap is larger. Use this version only for a specific
// comparison/test, re-collect wheel_data afterward, and prefer the
// separately-measured right table (V9.14, git commit fe976c3) for
// production use.
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
  388.05f, 718.76f, 1044.48f, 1370.19f, 1695.91f,
  2021.62f, 2347.33f, 2673.05f, 2998.76f, 3324.47f
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
// Odometry speed from the ODOM_WINDOW_MS window (no EMA) - published in the
// /wheel/odom twist and used for pose integration and slip detection.
float odom_win_speed_l = 0.0f;
float odom_win_speed_r = 0.0f;

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
unsigned long last_imu_pub_time = 0;

// ============================================================
// IMU (MPU6050) - HAND-ROLLED BIT-BANGED I2C
//
// PB7 (SDA) and PC13 (SCL) were chosen from what's actually broken out on
// this board's headers - both sit on header P4, so both IMU wires land on
// the same connector. They don't share a hardware I2C peripheral (and
// PC13 isn't an I2C pin on this chip at all), which the STM32 HAL's Wire
// can't bridge across - but that's fine here since this is bit-banged:
// it just toggles two arbitrary GPIOs, so no peripheral pairing is
// required. (SCL started out on PB10, on a different header (P3) shared
// with this board's nRF24L01 footprint - moved to PC13 for simpler
// wiring and to free PB10 back up for that.)
//
// Not using Adafruit_MPU6050/Wire: Adafruit_BusIO's I2CDevice hard-
// requires a real TwoWire*, and the only pre-built software-I2C Arduino
// library available (testato/SoftwareWire) is AVR-only - it pokes raw
// AVR port registers directly and doesn't compile for STM32 at all. So
// this talks to the MPU6050 registers directly instead.
// ============================================================
#define IMU_SDA_PIN PB7
#define IMU_SCL_PIN PC13
#define MPU6050_ADDR      0x68
#define MPU6050_REG_WHOAMI      0x75
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
// AFS_SEL=2 (+-8g, 4096 LSB/g) / FS_SEL=1 (+-500 dps, 65.5 LSB/(deg/s)) -
// matches the ranges used on the (now retired) Arduino Mega IMU prototype.
#define MPU6050_ACCEL_LSB_PER_G   4096.0f
#define MPU6050_GYRO_LSB_PER_DPS  65.5f
// Yaw-rate scale correction (datasheet sensitivity is only +-3%). Measure
// with amr_calib_panel.py section 4 (gyro_scale = real angle / integrated
// angle, rotating the robot against a floor mark - wheel slip has no
// effect on it). 1.0 = uncalibrated.
const float GYRO_SCALE_Z = 1.0084f;  // calibrated 2026-10-02: 5-turn hand rotation, CW+CCW mean 1785.0 deg vs 1800 deg

bool mpu_ok = false;

static inline void i2c_sda_release() { pinMode(IMU_SDA_PIN, INPUT_PULLUP); }
static inline void i2c_sda_low()     { pinMode(IMU_SDA_PIN, OUTPUT); digitalWrite(IMU_SDA_PIN, LOW); }
static inline void i2c_scl_release() { pinMode(IMU_SCL_PIN, INPUT_PULLUP); }
static inline void i2c_scl_low()     { pinMode(IMU_SCL_PIN, OUTPUT); digitalWrite(IMU_SCL_PIN, LOW); }

void i2c_start()
{
  i2c_sda_release(); i2c_scl_release(); delayMicroseconds(3);
  i2c_sda_low(); delayMicroseconds(3);
  i2c_scl_low(); delayMicroseconds(3);
}

void i2c_stop()
{
  i2c_sda_low(); delayMicroseconds(3);
  i2c_scl_release(); delayMicroseconds(3);
  i2c_sda_release(); delayMicroseconds(3);
}

// Returns true on ACK
bool i2c_write_byte(uint8_t b)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (b & 0x80) i2c_sda_release(); else i2c_sda_low();
    b <<= 1;
    delayMicroseconds(3);
    i2c_scl_release(); delayMicroseconds(3);
    i2c_scl_low();
  }
  i2c_sda_release(); delayMicroseconds(3);
  i2c_scl_release(); delayMicroseconds(3);
  bool ack = (digitalRead(IMU_SDA_PIN) == LOW);
  i2c_scl_low();
  return ack;
}

uint8_t i2c_read_byte(bool ack)
{
  uint8_t b = 0;
  i2c_sda_release();
  for (uint8_t i = 0; i < 8; i++)
  {
    delayMicroseconds(3);
    i2c_scl_release(); delayMicroseconds(3);
    b = (b << 1) | (digitalRead(IMU_SDA_PIN) == HIGH ? 1 : 0);
    i2c_scl_low();
  }
  if (ack) i2c_sda_low(); else i2c_sda_release();
  delayMicroseconds(3);
  i2c_scl_release(); delayMicroseconds(3);
  i2c_scl_low();
  i2c_sda_release();
  return b;
}

bool mpu6050_write_reg(uint8_t reg, uint8_t val)
{
  i2c_start();
  bool ok = i2c_write_byte(MPU6050_ADDR << 1);
  ok = ok && i2c_write_byte(reg);
  ok = ok && i2c_write_byte(val);
  i2c_stop();
  return ok;
}

// Burst-reads `len` bytes starting at `reg` into buf[]. Returns true on ACK.
bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  i2c_start();
  bool ok = i2c_write_byte(MPU6050_ADDR << 1);
  ok = ok && i2c_write_byte(reg);
  if (!ok) { i2c_stop(); return false; }

  i2c_start();
  ok = i2c_write_byte((MPU6050_ADDR << 1) | 0x01);
  if (!ok) { i2c_stop(); return false; }

  for (uint8_t i = 0; i < len; i++) buf[i] = i2c_read_byte(i < len - 1);
  i2c_stop();
  return true;
}

// Bus recovery: an STM32 reset (e.g. re-flash) mid-read leaves the MPU
// still driving SDA low partway through a byte, and every later START then
// fails - mpu6050_init() returned false and /imu/data published all zeros
// until a power cycle. Clock SCL up to 9 times until the slave releases
// SDA, then issue a STOP (standard I2C bus-clear procedure).
void i2c_bus_recover()
{
  i2c_sda_release(); i2c_scl_release(); delayMicroseconds(5);
  for (uint8_t i = 0; i < 9 && digitalRead(IMU_SDA_PIN) == LOW; i++)
  {
    i2c_scl_low(); delayMicroseconds(5);
    i2c_scl_release(); delayMicroseconds(5);
  }
  i2c_scl_low(); delayMicroseconds(5);
  i2c_stop();
}

bool mpu6050_init()
{
  i2c_bus_recover();

  uint8_t whoami = 0;
  if (!mpu6050_read_regs(MPU6050_REG_WHOAMI, &whoami, 1)) return false;
  // 0x68 = MPU6050, 0x69 = some MPU6050 clones, 0x70 = MPU6500, 0x71 =
  // MPU9250 - all register-compatible with the reads/writes used here.
  // (This board's "GY-521" breakout reads back 0x70 - it's actually an
  // MPU6500 under the same silkscreen/pinout, a common substitution.)
  if (whoami != 0x68 && whoami != 0x69 && whoami != 0x70 && whoami != 0x71) return false;

  // Wake from sleep with CLKSEL=1 (PLL referenced to the gyro X MEMS
  // oscillator) rather than 0 (internal 20MHz RC). The datasheet
  // recommends the PLL for gyro stability; suspected cause of the
  // isolated per-axis gyro spikes seen while stationary (accel clean).
  if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x10)) return false;  // +-8g
  if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x08)) return false;   // +-500 dps
  // DLPF_CFG=4: ~21Hz accel bandwidth / ~20Hz gyro bandwidth - same
  // setting the retired Arduino Mega prototype used (MPU6050_BAND_21_HZ).
  // Without this the chip defaults to its widest (~260Hz) bandwidth,
  // passing through most of the vibration noise from being mounted next
  // to the motor driver circuitry on this same board.
  if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x04)) return false;
  return true;
}

// ============================================================
// IMU BIAS AUTO-CALIBRATION (gyro + accel)
//
// Averages raw (pre-unit-conversion) gyro/accel counts over `samples`
// reads (one combined 14-byte burst read per sample, so this doesn't cost
// extra I2C traffic over gyro-only calibration) to find each axis's
// zero-rate/zero-g offset, then those averages are subtracted from every
// live reading in publish_imu().
//
// Gyro: a stationary sensor should read exactly 0 rad/s on all 3 axes,
// so the raw average IS the bias directly. Without correcting this, even
// a small constant bias (e.g. 0.01 rad/s) integrates into large heading
// drift over time once fused into the EKF - see ekf.yaml's odom0_config
// comment on why heading accuracy matters here.
//
// Accel: a stationary, LEVEL sensor should read (0, 0, +1g) - gravity is
// real, not bias, so only the DEVIATION from that is bias. x/y biases are
// just their raw averages (expected 0); z's bias is (raw average - the
// raw count representing 1g), so the correction still leaves gravity in
// the published reading rather than zeroing it out. This was added after
// a stationary, level reading came back as x=1.14 y=-0.08 z=10.67 m/s^2 -
// magnitude 10.73 vs the true 9.81 m/s^2 of gravity, which is too large
// to be explained by mounting tilt alone (tilt redistributes gravity
// across axes but can't change the total magnitude), confirming a real
// per-axis sensor offset.
//
// Only called from setup(), before the ROS executor/agent connection
// exists (state machine starts at WAITING_AGENT after this returns) -
// so no cmd_vel can reach the motors while this runs. The robot MUST be
// stationary AND level for the ~1s this takes: gyro calibration only
// needs "stationary", but accel calibration also needs "level", or the
// z-axis bias above will absorb the tilt error too.
// ============================================================
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;

// ============================================================
// WHEEL SLIP DETECTION (approach A - detect only, don't correct)
//
// Compares wheel-odometry's own yaw rate estimate (`w` in
// update_and_publish_odom(), computed purely from encoder counts) against
// the gyro's independently-measured angular_velocity.z. Encoders can't
// tell the difference between "wheel turned because the robot turned"
// and "wheel turned because it slipped in place" - the gyro can, since
// it measures the body's actual rotation regardless of what the wheels
// are doing. A sustained mismatch between the two means the wheels
// aren't tracking the ground truth motion, i.e. slip.
//
// This only reports the condition (wheel/slip_detected) - it does NOT
// feed back into motor_control_process() to correct anything. Closing
// that loop would make the gyro a safety-critical input to motion
// control itself (a bad gyro reading could then directly distort motor
// commands, not just localization) - see the mounting-height/motor-
// control discussion this was scoped down from. If detections in
// practice show slip is frequent enough to matter, that's a separate,
// more involved change to design and test carefully.
// ============================================================
float latest_gyro_yaw_rate_rad_s = 0.0f;
bool gyro_yaw_rate_valid = false;  // true once publish_imu() has stored a real reading

const float SLIP_YAW_RATE_THRESHOLD_RAD_S = 0.15f;  // ~8.6 deg/s - above normal filter-lag mismatch between the two signals
const unsigned long SLIP_DEBOUNCE_MS = 300;          // mismatch must persist this long before latching true (avoids false positives from turning transients)
unsigned long slip_mismatch_start = 0;
bool slip_mismatch_active = false;
bool wheel_slip_detected = false;

// ============================================================
// CONTINUOUS GYRO BIAS TRACKING (ZUPT - zero-rate update)
//
// calibrate_imu_bias() only measures bias once at boot, while the chip is
// still cold. MEMS gyro zero-rate offset drifts with die temperature -
// as the chip (and the motor driver board it shares) warms up during
// operation, the true bias moves away from that one boot-time snapshot,
// showing up as a residual angular_velocity.z that never quite settles
// to 0 even at rest (observed ~-0.027 rad/s / -1.5 deg/s after boot
// calibration - enough to integrate into real heading drift if left
// uncorrected over an idle period).
//
// Fix: whenever the robot is genuinely commanded-and-measured stationary
// (same condition update_and_publish_odom() already uses to zero
// odom_speed_l/r - see robot_stationary below), nudge each gyro bias
// slowly toward the current raw reading (true angular velocity is 0 at
// rest, so the raw reading IS the current bias). ZUPT_ALPHA is small so
// this tracks slow thermal drift over tens of seconds, not noise on a
// single sample.
//
// robot_stationary is keyed off target_speed_l/r (what was COMMANDED),
// not just measured speed, so a robot forced to a sudden stop (e.g. an
// obstacle) doesn't get misread as "idle, safe to recalibrate" - matches
// the same commanded-vs-measured distinction STOP_TARGET_THRESHOLD/
// STOP_SPEED_THRESHOLD already draw elsewhere in this file.
// ============================================================
bool robot_stationary = false;
const float ZUPT_ALPHA = 0.004f; // time constant ~= 1/(ZUPT_ALPHA * 50Hz) = ~5s (IMU_PUBLISH_PERIOD_MS)
// Only update bias while |raw - bias| < ~0.5 deg/s on every axis (65.5 LSB
// per deg/s at FS_SEL=1). Thermal drift is far slower than this; real
// rotation is far faster - see the gate in publish_imu().
const float ZUPT_GYRO_GATE_LSB = 0.5f * MPU6050_GYRO_LSB_PER_DPS;
const uint8_t ZUPT_QUIET_SAMPLES = 25;  // 0.5 s at 50 Hz
uint8_t zupt_quiet_count = 0;
// Recovery for a bias that is already off by more than the gate above
// (e.g. boot calibration taken while the gyro was still settling - the
// narrow gate alone measured 128 deg/min of drift at rest that it could
// never correct). While the wheels are stationary, collect a window of
// samples; if every axis stays within ZUPT_SNAP_RANGE_LSB peak-to-peak (a
// held, still sensor - hand rotation is never this steady) and its mean is
// within ZUPT_SNAP_MAX_LSB of the current bias, snap the bias to that mean.
const uint16_t ZUPT_SNAP_SAMPLES = 100;                                  // 2 s at 50 Hz
const float ZUPT_SNAP_RANGE_LSB = 0.6f * MPU6050_GYRO_LSB_PER_DPS;       // peak-to-peak (sensor noise ~0.3 p-p)
const float ZUPT_SNAP_MAX_LSB = 5.0f * MPU6050_GYRO_LSB_PER_DPS;         // plausible bias error
uint16_t zupt_snap_count = 0;
float zupt_snap_sum[3], zupt_snap_min[3], zupt_snap_max[3];

void calibrate_imu_bias(uint16_t samples)
{
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  uint16_t got = 0;
  uint8_t raw[14];

  for (uint16_t i = 0; i < samples; i++)
  {
    if (mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, 14))
    {
      sum_ax += (int16_t)((raw[0] << 8) | raw[1]);
      sum_ay += (int16_t)((raw[2] << 8) | raw[3]);
      sum_az += (int16_t)((raw[4] << 8) | raw[5]);
      sum_gx += (int16_t)((raw[8]  << 8) | raw[9]);
      sum_gy += (int16_t)((raw[10] << 8) | raw[11]);
      sum_gz += (int16_t)((raw[12] << 8) | raw[13]);
      got++;
    }
    delay(3);
  }

  if (got == 0) return;  // leave biases at 0 rather than divide by zero
  gyro_bias_x = (float)sum_gx / got;
  gyro_bias_y = (float)sum_gy / got;
  gyro_bias_z = (float)sum_gz / got;
  accel_bias_x = (float)sum_ax / got;
  accel_bias_y = (float)sum_ay / got;
  accel_bias_z = ((float)sum_az / got) - MPU6050_ACCEL_LSB_PER_G;
}

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
rcl_publisher_t left_pulse_publisher;
rcl_publisher_t right_pulse_publisher;
rcl_publisher_t raw_distance_publisher;
rcl_publisher_t slip_publisher;
rcl_publisher_t imu_publisher;

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Float32 msg_left_cmd;
std_msgs__msg__Float32 msg_right_cmd;
std_msgs__msg__Bool msg_pulse_reset;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32 left_pulse_msg;
std_msgs__msg__Int32 right_pulse_msg;
std_msgs__msg__Float32 raw_distance_msg;
std_msgs__msg__Bool slip_msg;
sensor_msgs__msg__Imu imu_msg;

// Cumulative signed distance integrated straight from raw_speed_l/r (the
// unfiltered, VELOCITY_WINDOW_MS-averaged encoder speed), NOT from
// odom_speed_l/r. odom_speed_l/r goes through ODOM_FILTER_ALPHA (a heavy
// 0.08 low-pass) before being published on wheel/odom and fused by the
// EKF, which lags real distance during any speed transient (e.g. the
// accel ramp at the start of a move) - see move_forward_node.py's
// docstring for the overshoot this caused. This value skips that filter
// entirely so callers who need close-to-real-time distance (like
// move_forward_node.py) don't inherit that lag. Monotonically-signed
// running total from boot, not reset between moves - callers diff against
// their own captured start value.
float raw_distance_traveled = 0.0f;

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

// Periodic re-sync: the STM32 clock measured ~46 ms/min (~775 ppm) fast
// against the Jetson, so odom/IMU header stamps drifted into the future
// (2026-09-25 measurement, `now - stamp` -115 ms after 150 s) when synced
// only at connect + 3 s. Re-syncing every TIME_SYNC_PERIOD_MS bounds the
// error to ~4 ms. rmw_uros_sync_session() blocks the main loop (motor PID
// keeps its last DAC output meanwhile), so it uses a SHORT timeout here:
// a slow/absent agent must not stall control for the 1000 ms the
// connect-time sync allows. A failed sync is simply retried next period.
const unsigned long TIME_SYNC_PERIOD_MS  = 5000;
const int           TIME_SYNC_TIMEOUT_MS = 100;
unsigned long last_time_sync_ms = 0;

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
void update_wheel_dir();
void update_startup_balance();
float get_startup_left_correction();
float get_startup_right_correction();
void motor_control_process();
void update_and_publish_odom();
void publish_pulse_counts();
void publish_imu();
int16_t median3(int16_t a, int16_t b, int16_t c);
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
  if (rclc_publisher_init_default(&left_pulse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "wheel/left/pulse_count") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&right_pulse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "wheel/right/pulse_count") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&raw_distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel/raw_distance") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&slip_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "wheel/slip_detected") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data") != RCL_RET_OK) return false;

  // EXECUTOR
  if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_left_cmd, &msg_left_cmd, left_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_right_cmd, &msg_right_cmd, right_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_pulse_reset, &msg_pulse_reset, pulse_reset_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // ODOM FRAME NAMES
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  // ODOM TWIST COVARIANCE - was never set (all zero), which robot_localization
  // treats as near-infinite confidence, so the EKF effectively ignored the
  // gyro's vyaw. Row-major 6x6 (vx, vy, vz, wx, wy, wz). vyaw is set much
  // looser than the gyro (angular_velocity_covariance z = 0.001) so heading
  // follows the gyro when one wheel slips; vx stays tight - wheels are the
  // only forward-velocity source.
  odom_msg.twist.covariance[0]  = 0.002;  // vx
  odom_msg.twist.covariance[7]  = 0.002;  // vy (diff drive: ~0, not fused)
  odom_msg.twist.covariance[14] = 1e6;    // vz
  odom_msg.twist.covariance[21] = 1e6;    // wx
  odom_msg.twist.covariance[28] = 1e6;    // wy
  odom_msg.twist.covariance[35] = 0.05;   // vyaw - 50x looser than the gyro

  // IMU FRAME NAME + orientation covariance[0] = -1 means "orientation not
  // computed" (REP 145) - accel/gyro readings themselves are real (see
  // publish_imu()), only orientation (which this sensor can't produce
  // without its own separate fusion) is left unset.
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  imu_msg.orientation_covariance[0] = -1.0;

  // angular_velocity_covariance / linear_acceleration_covariance default
  // to all-zero (sensor_msgs__msg__Imu's generated struct zero-inits
  // float arrays) if never set - and an all-zero covariance matrix is
  // NOT "no noise", it's invalid input to ekf_node's Kalman gain
  // computation (which divides by the measurement covariance). This was
  // exactly what caused /odom to stop publishing entirely after imu0 was
  // enabled in ekf.yaml: the singular covariance produced NaNs that
  // propagated through the filter's state and then got silently dropped
  // by tf2 (which refuses to broadcast a NaN transform), while
  // ekf_filter_node itself kept running without crashing - "ros2 node
  // list" still showed it alive even though it had stopped producing
  // any output.
  //
  // Diagonal-only (3x3 row-major, so indices 0/4/8), off-diagonal terms
  // left at 0 (no known cross-axis correlation). Values are order-of-
  // magnitude estimates, not measured: gyro variance is low since it's
  // bias-calibrated + DLPF-filtered + median-of-3 filtered (see
  // publish_imu()); accel variance is higher since it's only bias-
  // calibrated (not full 6-point scale/cross-axis calibrated) - matches
  // why ekf.yaml's imu0_config doesn't fuse accel at all, this value is
  // just here so the field isn't a singular matrix for anything that
  // reads it.
  imu_msg.angular_velocity_covariance[0] = 0.001;  // x
  imu_msg.angular_velocity_covariance[4] = 0.001;  // y
  imu_msg.angular_velocity_covariance[8] = 0.001;  // z - the one ekf.yaml actually fuses
  imu_msg.linear_acceleration_covariance[0] = 0.05;  // x
  imu_msg.linear_acceleration_covariance[4] = 0.05;  // y
  imu_msg.linear_acceleration_covariance[8] = 0.05;  // z

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
  rosidl_runtime_c__String__fini(&imu_msg.header.frame_id);

  (void) rcl_publisher_fini(&odom_publisher, &node);
  (void) rcl_publisher_fini(&left_pulse_publisher, &node);
  (void) rcl_publisher_fini(&right_pulse_publisher, &node);
  (void) rcl_publisher_fini(&raw_distance_publisher, &node);
  (void) rcl_publisher_fini(&slip_publisher, &node);
  (void) rcl_publisher_fini(&imu_publisher, &node);

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
  odom_win_speed_l = 0.0f; odom_win_speed_r = 0.0f;
  prev_error_l = 0.0f; prev_error_r = 0.0f;
  integral_l = 0.0f; integral_r = 0.0f;
  desired_dac_l = 0.0f; desired_dac_r = 0.0f;
  dac_output_l = 0; dac_output_r = 0;
  current_target_l = 0.0f; current_target_r = 0.0f;
  dir_l = 1; dir_r = 1;
  dac_zero_since_l = 0; dac_zero_since_r = 0;

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
// WHEEL DIRECTION LATCH - see the declaration comment above
// ============================================================
static int next_wheel_dir(int dir, float target, int dac_output, float raw_speed,
                          unsigned long &dac_zero_since, unsigned long now)
{
  if (dac_output != 0) { dac_zero_since = 0; return dir; }
  if (dac_zero_since == 0) dac_zero_since = now;
  int want = (target > 0.001f) ? 1 : (target < -0.001f) ? -1 : dir;
  if (want == dir) return dir;
  bool stopped = fabs(raw_speed) < STOP_SPEED_THRESHOLD;
  bool waited = (now - dac_zero_since) > DIR_CHANGE_TIMEOUT_MS;
  return (stopped || waited) ? want : dir;
}

void update_wheel_dir()
{
  unsigned long now = millis();
  dir_l = next_wheel_dir(dir_l, current_target_l, dac_output_l, raw_speed_l, dac_zero_since_l, now);
  dir_r = next_wheel_dir(dir_r, current_target_r, dac_output_r, raw_speed_r, dac_zero_since_r, now);
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
  bool left_forward = dir_l > 0;
  bool right_forward = dir_r > 0;

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
    odom_win_speed_l = 0.0f;
    odom_win_speed_r = 0.0f;
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

    float distance_l = ((float)delta_pulse_l / ENCODER_PPR) * wheel_circumference_l;
    float distance_r = ((float)delta_pulse_r / ENCODER_PPR) * wheel_circumference_r;

    float dt = elapsed_ms / 1000.0f;
    if (dt > 0.0f)
    {
      raw_speed_l = distance_l / dt;
      raw_speed_r = distance_r / dt;
    }
  }

  // Odometry window: newest slot (k ticks back) that is >= ODOM_WINDOW_MS
  // old. If none qualifies yet (start-up / long tick gaps) keep last value.
  for (int k = 1; k < VELOCITY_HISTORY_SIZE; k++)
  {
    int idx = (history_index - k + VELOCITY_HISTORY_SIZE) % VELOCITY_HISTORY_SIZE;
    unsigned long age_ms = now - time_history[idx];
    if (age_ms >= ODOM_WINDOW_MS)
    {
      float inv_dt = 1000.0f / (float)age_ms;
      odom_win_speed_l = ((float)(velocity_pulse_l - pulse_history_l[idx]) / ENCODER_PPR) * wheel_circumference_l * inv_dt;
      odom_win_speed_r = ((float)(velocity_pulse_r - pulse_history_r[idx]) / ENCODER_PPR) * wheel_circumference_r * inv_dt;
      break;
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
  // RUN/EN: LOW = run, HIGH = stop/disable driver.
  // ตัด RUN/EN เมื่อ DAC ของล้อนั้น slew ลงถึง 0 แล้วเท่านั้น (รอเบรกจบก่อน ไม่ตัดกะทันหันตาม cmd_vel)
  digitalWrite(L_RUN_PIN, (dac_output_l == 0) ? HIGH : LOW);
  digitalWrite(R_RUN_PIN, (dac_output_r == 0) ? HIGH : LOW);

  digitalWrite(L_FR_PIN, (dir_l > 0) ? HIGH : LOW);
  digitalWrite(R_FR_PIN, (dir_r > 0) ? LOW : HIGH);
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
  update_wheel_dir();

  long count_l = 0, count_r = 0;
  read_encoder_counts(count_l, count_r);
  update_velocity_estimator(count_l, count_r, now);

  update_control_filters();
  update_odom_filters();

  update_startup_balance();
  update_left_launch();
  update_right_launch();

  robot_stationary = false;
  if (fabs(target_speed_l) < STOP_TARGET_THRESHOLD && fabs(target_speed_r) < STOP_TARGET_THRESHOLD)
  {
    if (fabs(raw_speed_l) < STOP_SPEED_THRESHOLD) { odom_speed_l = 0.0f; odom_win_speed_l = 0.0f; }
    if (fabs(raw_speed_r) < STOP_SPEED_THRESHOLD) { odom_speed_r = 0.0f; odom_win_speed_r = 0.0f; }
    // Also true stationary for ZUPT gyro bias tracking - see its
    // declaration comment - same commanded+measured check as above.
    if (fabs(raw_speed_l) < STOP_SPEED_THRESHOLD && fabs(raw_speed_r) < STOP_SPEED_THRESHOLD)
      robot_stationary = true;
  }

  desired_dac_l = compute_left_dac(current_target_l, control_speed_l, dt);
  desired_dac_r = compute_right_dac(current_target_r, control_speed_r, dt);

  // Command points the other way than the latched direction: bring this
  // wheel's DAC to 0 first (STOP slew below); update_wheel_dir() flips the
  // direction once it is at 0 and the wheel has stopped.
  bool reversing_l = (current_target_l * (float)dir_l) < 0.0f;
  bool reversing_r = (current_target_r * (float)dir_r) < 0.0f;
  if (reversing_l) { desired_dac_l = 0.0f; integral_l = 0.0f; }
  if (reversing_r) { desired_dac_r = 0.0f; integral_r = 0.0f; }

  // V9.8: speed-proportional normal slew
  float slew_l = SLEW_BASE + (SLEW_MAX - SLEW_BASE) * constrain(fabs(current_target_l) / SLEW_FULL_SPEED, 0.0f, 1.0f);
  float slew_r = SLEW_BASE + (SLEW_MAX - SLEW_BASE) * constrain(fabs(current_target_r) / SLEW_FULL_SPEED, 0.0f, 1.0f);

  // LEFT SLEW
  float dac_step_l = (fabs(target_speed_l) < STOP_TARGET_THRESHOLD || reversing_l) ? STOP_DAC_STEP_L :
                     (left_launch_active) ? LAUNCH_DAC_STEP_L : slew_l;
  dac_output_l = apply_dac_slew(desired_dac_l, dac_output_l, dac_step_l);

  // RIGHT SLEW
  float dac_step_r = (fabs(target_speed_r) < STOP_TARGET_THRESHOLD || reversing_r) ? STOP_DAC_STEP_R :
                     (right_launch_active) ? RIGHT_LAUNCH_DAC_STEP : slew_r;
  dac_output_r = apply_dac_slew(desired_dac_r, dac_output_r, dac_step_r);

  // RAW (UNFILTERED-BY-ODOM_FILTER_ALPHA) DISTANCE
  // Same integration as the odometry below, but from raw_speed_l/r instead
  // of odom_speed_l/r - see raw_distance_traveled's declaration comment.
  float v_raw = (raw_speed_r + raw_speed_l) / 2.0f;
  raw_distance_traveled += v_raw * dt;
  raw_distance_msg.data = raw_distance_traveled;
  rcl_publish(&raw_distance_publisher, &raw_distance_msg, NULL);

  // ODOMETRY CALCULATION - from the ODOM_WINDOW_MS windowed speed with NO
  // EMA on top. ODOM_FILTER_ALPHA=0.08 at CONTROL_PERIOD_MS=50ms gives a
  // ~625ms EMA time constant, which made pose_theta and the /wheel/odom
  // twist lag real motion by up to ~1-2s (the robot visibly finished
  // turning well before its map icon caught up; linear.x still carried
  // that lag until now). v and w both use the same shorter window so pose,
  // twist and the gyro slip check are mutually consistent. PID feedback
  // is unchanged (raw_speed_l/r -> control_speed_l/r).
  float v = (odom_win_speed_r + odom_win_speed_l) / 2.0f;
  float w = (odom_win_speed_r - odom_win_speed_l) / wheel_sep;

  // WHEEL SLIP DETECTION - see its declaration comment for why this
  // compares against the gyro instead of trusting encoders alone.
  if (gyro_yaw_rate_valid)
  {
    if (fabs(w - latest_gyro_yaw_rate_rad_s) > SLIP_YAW_RATE_THRESHOLD_RAD_S)
    {
      if (!slip_mismatch_active) { slip_mismatch_active = true; slip_mismatch_start = now; }
      else if (now - slip_mismatch_start > SLIP_DEBOUNCE_MS) wheel_slip_detected = true;
    }
    else
    {
      slip_mismatch_active = false;
      wheel_slip_detected = false;
    }
  }
  else
  {
    slip_mismatch_active = false;
    wheel_slip_detected = false;
  }
  // Published every odom cycle (not just on change) so a late subscriber
  // or a dropped message can't leave a stale "true" latched downstream.
  slip_msg.data = wheel_slip_detected;
  rcl_publish(&slip_publisher, &slip_msg, NULL);

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

// Returns the median of 3 values - used by publish_imu() to reject a
// single bit-bang I2C read glitched by encoder-ISR timing jitter (see
// that function's comment) without needing to track any state between
// calls.
int16_t median3(int16_t a, int16_t b, int16_t c)
{
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

// ============================================================
// PUBLISH IMU
//
// Rate-limited to 20Hz like publish_pulse_counts() above, on a hand-
// rolled bit-banged I2C bus (PB7/PC13 - see the IMU_SDA_PIN/IMU_SCL_PIN
// comment above) - bit-banging is slower and
// burns more CPU per byte than hardware I2C (and this reads 3x per
// publish for the median filter below), so keeping this at 20Hz instead
// of polling every loop() iteration matters more here than it would with
// a real I2C peripheral, to avoid adding jitter to the PID/encoder timing.
//
// orientation_covariance[0] stays at -1 (REP 145: "orientation not
// computed") because the MPU6050/MPU6500 has no onboard sensor fusion -
// only raw accel/gyro are filled in below. ekf_node fuses those directly.
// If mpu6050_init() failed at setup() (sensor not wired/detected), this
// leaves linear_acceleration/angular_velocity at their last (zero-
// initialized) values rather than publishing stale/garbage reads.
// ============================================================
// IMU publish rate: 50 Hz (was 20 Hz) - halves the sample-and-hold lag
// (~25 -> ~10 ms) and removes most gyro aliasing (DLPF ~20 Hz vs 10 Hz
// Nyquist at 20 Hz). DLPF is left at ~20 Hz on purpose: a wider DLPF
// would alias again at 50 Hz sampling (Nyquist 25 Hz).
const unsigned long IMU_PUBLISH_PERIOD_MS = 20;

// CPU cost control: bit-banged I2C makes one 14-byte read ~1-1.5 ms
// (estimate), so the old median-of-3 on EVERY publish would cost ~20% of
// the main loop at 50 Hz. Instead take ONE read and accept it only if it
// is close to the previous accepted sample (a real 20 ms change is small;
// the corruption seen so far were huge single-bit-in-high-byte jumps).
// If it jumps beyond the gates, fall back to the full median-of-3 (2 more
// reads), so glitch protection is unchanged for the cases that matter and
// CPU stays ~at the old 20 Hz level. Residual risk: a low-bit flip below
// the gate passes for one 20 ms sample (bounded error <= gate, ~7.6 deg/s
// gyro / 0.25 g accel) - tighten the gates or drop to median-of-3 always
// if such spikes appear.
const int16_t IMU_GATE_GYRO_COUNTS  = 500;   // ~7.6 deg/s per 20 ms
const int16_t IMU_GATE_ACCEL_COUNTS = 1024;  // ~0.25 g per 20 ms
int16_t imu_prev_raw[6];                     // ax ay az gx gy gz, last accepted
bool imu_prev_valid = false;

void publish_imu()
{
  unsigned long now = millis();
  if (now - last_imu_pub_time < IMU_PUBLISH_PERIOD_MS) return;
  // Advance by whole periods (no drift from loop jitter); resync if we
  // fell more than one period behind (e.g. blocked by a time sync).
  if (now - last_imu_pub_time >= 2 * IMU_PUBLISH_PERIOD_MS) last_imu_pub_time = now;
  else last_imu_pub_time += IMU_PUBLISH_PERIOD_MS;

  int64_t time_ms = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = time_ms / 1000;
  imu_msg.header.stamp.nanosec = (time_ms % 1000) * 1000000;

  // No IMU (init failed): publish nothing rather than an all-zero message -
  // with vyaw fused in ekf.yaml, zeros would be trusted as "not rotating"
  // and corrupt heading on every turn. Without /imu/data the EKF falls back
  // to wheel odometry alone.
  if (!mpu_ok) return;

  {
    // Sanity-check + median-of-3 against bit-bang I2C glitches: an
    // encoder ISR (L_HAL_ISR/R_HAL_ISR) firing mid-transaction can skew
    // this driver's microsecond-level bit timing and corrupt a byte
    // without failing the ACK check. This showed up as brief nonsense
    // spikes (angular_velocity.y ~9.6 rad/s, then x ~-6.6, then x ~-5.3
    // while stationary) - and that last one nearly slipped past a single
    // absolute-threshold check (20000 raw / ~305deg/s), because a single
    // bit flipping near the middle of a 16-bit reading lands the
    // corrupted value in a fairly consistent, not-always-extreme range -
    // an absolute cutoff alone isn't reliable against that.
    //
    // Reading 3 times in a row (~1ms apart, well within the 50ms publish
    // period) and taking the per-axis median instead is far more robust:
    // a stuck-timing glitch on any ONE of the 3 reads gets outvoted by
    // the other 2 good ones, whereas the same bit corrupting in exactly
    // the same way on 2+ of 3 back-to-back reads is effectively
    // impossible for a timing-jitter-driven fault like this.
    //
    // Rather than disabling interrupts around the I2C transfer (which
    // would risk dropping encoder pulses and hurt PID accuracy - not
    // worth it here), a still-implausible median (e.g. bus genuinely
    // disconnected) falls back to republishing the last good imu_msg
    // values instead (with a fresh timestamp).
    int16_t ax3[3], ay3[3], az3[3], gx3[3], gy3[3], gz3[3];
    uint8_t got = 0;
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    bool have = false;
    for (uint8_t i = 0; i < 3; i++)
    {
      uint8_t raw[14];
      if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, 14)) continue;
      ax3[got] = (raw[0] << 8) | raw[1];
      ay3[got] = (raw[2] << 8) | raw[3];
      az3[got] = (raw[4] << 8) | raw[5];
      // raw[6..7] = temperature, unused here
      gx3[got] = (raw[8] << 8) | raw[9];
      gy3[got] = (raw[10] << 8) | raw[11];
      gz3[got] = (raw[12] << 8) | raw[13];
      got++;

      // Fast path: first read close to the previous accepted sample ->
      // accept it alone (see IMU_GATE_* comment).
      if (i == 0 && imu_prev_valid &&
          abs(ax3[0] - imu_prev_raw[0]) < IMU_GATE_ACCEL_COUNTS &&
          abs(ay3[0] - imu_prev_raw[1]) < IMU_GATE_ACCEL_COUNTS &&
          abs(az3[0] - imu_prev_raw[2]) < IMU_GATE_ACCEL_COUNTS &&
          abs(gx3[0] - imu_prev_raw[3]) < IMU_GATE_GYRO_COUNTS &&
          abs(gy3[0] - imu_prev_raw[4]) < IMU_GATE_GYRO_COUNTS &&
          abs(gz3[0] - imu_prev_raw[5]) < IMU_GATE_GYRO_COUNTS)
      {
        ax = ax3[0]; ay = ay3[0]; az = az3[0];
        gx = gx3[0]; gy = gy3[0]; gz = gz3[0];
        have = true;
        break;
      }
    }

    if (!have && got == 3)
    {
      ax = median3(ax3[0], ax3[1], ax3[2]);
      ay = median3(ay3[0], ay3[1], ay3[2]);
      az = median3(az3[0], az3[1], az3[2]);
      gx = median3(gx3[0], gx3[1], gx3[2]);
      gy = median3(gy3[0], gy3[1], gy3[2]);
      gz = median3(gz3[0], gz3[1], gz3[2]);
      have = true;
    }

    if (have)
    {
      const int16_t GYRO_RAW_SANITY_MAX = 20000;   // ~305 deg/s - normal operation shouldn't approach this
      const int16_t ACCEL_RAW_SANITY_MAX = 30000;  // ~7.3g - normal operation/vibration shouldn't approach this
      bool sane =
          abs(ax) < ACCEL_RAW_SANITY_MAX && abs(ay) < ACCEL_RAW_SANITY_MAX && abs(az) < ACCEL_RAW_SANITY_MAX &&
          abs(gx) < GYRO_RAW_SANITY_MAX  && abs(gy) < GYRO_RAW_SANITY_MAX  && abs(gz) < GYRO_RAW_SANITY_MAX;

      if (!sane)
      {
        imu_prev_valid = false;  // next publish must re-verify with median-of-3
        rcl_publish(&imu_publisher, &imu_msg, NULL);
        return;
      }
      imu_prev_raw[0] = ax; imu_prev_raw[1] = ay; imu_prev_raw[2] = az;
      imu_prev_raw[3] = gx; imu_prev_raw[4] = gy; imu_prev_raw[5] = gz;
      imu_prev_valid = true;

      // ZUPT: slowly track thermal drift while genuinely stationary -
      // see robot_stationary's declaration comment. Uses the clean
      // (sanity-checked, median-of-3) raw counts above, not raw byte
      // reads, so a glitched sample can't corrupt the bias estimate.
      //
      // Gated on the gyro itself also reading ~zero rate: robot_stationary
      // only proves the WHEELS aren't turning, but the body can still be
      // rotating (pushed/turned by hand, wheels skidding or lifted). Without
      // this gate ZUPT absorbed real rotation into the bias - measured hand
      // rotation of 5 turns lost 4-15% (more the slower it was turned).
      // ZUPT_QUIET_SAMPLES consecutive quiet samples are also required so
      // the start/end of a rotation (rate passing through zero) isn't used.
      bool gyro_quiet =
          fabsf(gx - gyro_bias_x) < ZUPT_GYRO_GATE_LSB &&
          fabsf(gy - gyro_bias_y) < ZUPT_GYRO_GATE_LSB &&
          fabsf(gz - gyro_bias_z) < ZUPT_GYRO_GATE_LSB;
      if (robot_stationary && gyro_quiet)
      {
        if (zupt_quiet_count < ZUPT_QUIET_SAMPLES) zupt_quiet_count++;
      }
      else
      {
        zupt_quiet_count = 0;
      }

      if (robot_stationary)
      {
        const float g[3] = {(float)gx, (float)gy, (float)gz};
        bool steady = true;
        for (uint8_t i = 0; i < 3; i++)
        {
          if (zupt_snap_count == 0) { zupt_snap_sum[i] = 0.0f; zupt_snap_min[i] = g[i]; zupt_snap_max[i] = g[i]; }
          zupt_snap_sum[i] += g[i];
          if (g[i] < zupt_snap_min[i]) zupt_snap_min[i] = g[i];
          if (g[i] > zupt_snap_max[i]) zupt_snap_max[i] = g[i];
          if (zupt_snap_max[i] - zupt_snap_min[i] > ZUPT_SNAP_RANGE_LSB) steady = false;
        }
        zupt_snap_count = steady ? zupt_snap_count + 1 : 0;
        if (zupt_snap_count >= ZUPT_SNAP_SAMPLES)
        {
          float mx = zupt_snap_sum[0] / zupt_snap_count;
          float my = zupt_snap_sum[1] / zupt_snap_count;
          float mz = zupt_snap_sum[2] / zupt_snap_count;
          if (fabsf(mx - gyro_bias_x) < ZUPT_SNAP_MAX_LSB &&
              fabsf(my - gyro_bias_y) < ZUPT_SNAP_MAX_LSB &&
              fabsf(mz - gyro_bias_z) < ZUPT_SNAP_MAX_LSB)
          {
            gyro_bias_x = mx; gyro_bias_y = my; gyro_bias_z = mz;
          }
          zupt_snap_count = 0;
        }
      }
      else
      {
        zupt_snap_count = 0;
      }

      if (zupt_quiet_count >= ZUPT_QUIET_SAMPLES)
      {
        gyro_bias_x += ZUPT_ALPHA * (gx - gyro_bias_x);
        gyro_bias_y += ZUPT_ALPHA * (gy - gyro_bias_y);
        gyro_bias_z += ZUPT_ALPHA * (gz - gyro_bias_z);
      }

      const float IMU_G_TO_MS2 = 9.80665f;
      const float IMU_DEG_TO_RAD = PI / 180.0f;

      // Subtract the boot-time offsets (calibrate_imu_bias()) before
      // converting to physical units - see that function's comment for
      // why accel's z-axis bias is relative to +1g rather than 0.
      //
      // x/y are then negated: the board is physically mounted rotated
      // 180deg about z relative to the robot's forward axis, so the raw
      // sensor's +x/+y point the opposite way from base_link's +x/+y.
      // z is unaffected by a rotation about the z axis itself. Since this
      // correction is applied here, imu_link's <origin rpy=".."> in the
      // URDF must stay "0 0 0" - don't also rotate it there, or the flip
      // gets applied twice and cancels out.
      imu_msg.linear_acceleration.x = -(((ax - accel_bias_x) / MPU6050_ACCEL_LSB_PER_G) * IMU_G_TO_MS2);
      imu_msg.linear_acceleration.y = -(((ay - accel_bias_y) / MPU6050_ACCEL_LSB_PER_G) * IMU_G_TO_MS2);
      imu_msg.linear_acceleration.z = ((az - accel_bias_z) / MPU6050_ACCEL_LSB_PER_G) * IMU_G_TO_MS2;
      imu_msg.angular_velocity.x = -(((gx - gyro_bias_x) / MPU6050_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD);
      imu_msg.angular_velocity.y = -(((gy - gyro_bias_y) / MPU6050_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD);
      imu_msg.angular_velocity.z = ((gz - gyro_bias_z) / MPU6050_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD * GYRO_SCALE_Z;

      // For wheel-slip detection (see its declaration comment) -
      // update_and_publish_odom() reads this to compare against its own
      // encoder-derived yaw rate.
      latest_gyro_yaw_rate_rad_s = imu_msg.angular_velocity.z;
      gyro_yaw_rate_valid = true;
    }
  }

  rcl_publish(&imu_publisher, &imu_msg, NULL);
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

  for (uint8_t attempt = 0; attempt < 5 && !mpu_ok; attempt++)
  {
    mpu_ok = mpu6050_init();
    if (!mpu_ok) delay(50);
  }
  // Let the gyro settle after wake (output drifts for the first ~1s) before
  // averaging, and average longer - a bad boot bias is no longer silently
  // pulled in by ZUPT's narrow gate (see ZUPT_SNAP_* for the recovery path).
  if (mpu_ok) { delay(1000); calibrate_imu_bias(600); }  // ~2s @ 3ms/sample - robot must be still AND level

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
        last_time_sync_ms = millis();
      }
      // Periodic re-sync every TIME_SYNC_PERIOD_MS (clock drift, see above)
      else if (time_resynced && (millis() - last_time_sync_ms) >= TIME_SYNC_PERIOD_MS)
      {
        (void) rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        last_time_sync_ms = millis();
      }

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      update_and_publish_odom();
      publish_pulse_counts();
      publish_imu();
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
