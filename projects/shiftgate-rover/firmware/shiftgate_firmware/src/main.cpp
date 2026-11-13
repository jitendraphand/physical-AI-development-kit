/*
 * ShiftGate Rover — ESP32 firmware.
 *
 * Default build: a USB-serial "line protocol" so you can debug motors,
 * encoders and the 3-beam scan with nothing but `pio device monitor`.
 *
 * Optional: compile with -DSHIFTGATE_MICROROS=1 once the micro-ROS
 * Arduino component is installed, and this sketch publishes
 *   geometry_msgs/Twist  (subscription on /cmd_vel)
 *   nav_msgs/Odometry    (on /odom, encoder-based)
 *   std_msgs/Float32MultiArray on /shiftgate/beams   [left_ir, front_m, right_ir]
 * and the laptop runs:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
 *
 * Pin map: see hardware/wiring.md.  Do not drive motors from the ESP32 5 V pin.
 */

#include <Arduino.h>

static const int PIN_AIN1 = 26, PIN_AIN2 = 27;
static const int PIN_BIN1 = 32, PIN_BIN2 = 33;
static const int PIN_NSLEEP = 25;
static const int PIN_ENC_L_A = 16, PIN_ENC_L_B = 17;
static const int PIN_ENC_R_A = 18, PIN_ENC_R_B = 19;
static const int PIN_US_TRIG = 23, PIN_US_ECHO = 4;
static const int PIN_IR_L = 34, PIN_IR_R = 35;

static const int PWM_FREQ = 20000;
static const int PWM_RES = 8;  // 0–255
static const int CH_A1 = 0, CH_A2 = 1, CH_B1 = 2, CH_B2 = 3;

// N20 100 RPM, 3 PPR, ~150:1 → ticks per wheel revolution.
static const float TICKS_PER_REV = 3.0f * 150.0f;
static const float WHEEL_RADIUS_M = 0.024f;
static const float WHEEL_SEPARATION_M = 0.102f;

volatile int32_t ticks_l = 0;
volatile int32_t ticks_r = 0;

void IRAM_ATTR onEncL() {
  int b = digitalRead(PIN_ENC_L_B);
  ticks_l += (b == digitalRead(PIN_ENC_L_A)) ? -1 : 1;
}
void IRAM_ATTR onEncR() {
  int b = digitalRead(PIN_ENC_R_B);
  ticks_r += (b == digitalRead(PIN_ENC_R_A)) ? 1 : -1;
}

static void motor_pwm(int ch_fwd, int ch_rev, float duty) {
  // duty in [-1, 1]
  duty = constrain(duty, -1.0f, 1.0f);
  uint8_t mag = (uint8_t)(fabsf(duty) * 255.0f);
  if (duty >= 0) {
    ledcWrite(ch_fwd, mag);
    ledcWrite(ch_rev, 0);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, mag);
  }
}

static void set_cmd_vel(float lin, float ang) {
  // Inverse kinematics, unicycle → wheel linear speeds.
  float v_l = lin - ang * (WHEEL_SEPARATION_M / 2.0f);
  float v_r = lin + ang * (WHEEL_SEPARATION_M / 2.0f);
  // Map 0.20 m/s ≈ full scale (100 RPM * 2π * 0.024 ≈ 0.25 m/s no-load).
  const float VMAX = 0.20f;
  motor_pwm(CH_A1, CH_A2, v_l / VMAX);
  motor_pwm(CH_B1, CH_B2, v_r / VMAX);
}

static float read_ultrasonic_m() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 25000UL);  // 25 ms timeout ≈ 4 m
  if (us == 0) return -1.0f;
  return (us * 1e-6f) * 343.0f / 2.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, HIGH);
  ledcSetup(CH_A1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_AIN1, CH_A1);
  ledcAttachPin(PIN_AIN2, CH_A2);
  ledcAttachPin(PIN_BIN1, CH_B1);
  ledcAttachPin(PIN_BIN2, CH_B2);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), onEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), onEncR, CHANGE);

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_IR_L, INPUT);
  pinMode(PIN_IR_R, INPUT);

  set_cmd_vel(0, 0);
  Serial.println("# ShiftGate firmware ready.  Commands: v <lin> <ang>   or   stop");
}

// Dead-simple odometry, integration in the body frame.
static float x = 0, y = 0, yaw = 0;
static int32_t last_l = 0, last_r = 0;
static uint32_t last_odom_ms = 0;

static void integrate_odom() {
  int32_t l = ticks_l, r = ticks_r;
  int32_t dl = l - last_l, dr = r - last_r;
  last_l = l;
  last_r = r;
  float dist_l = (2.0f * PI * WHEEL_RADIUS_M) * (dl / TICKS_PER_REV);
  float dist_r = (2.0f * PI * WHEEL_RADIUS_M) * (dr / TICKS_PER_REV);
  float ds = 0.5f * (dist_l + dist_r);
  float dth = (dist_r - dist_l) / WHEEL_SEPARATION_M;
  yaw += dth;
  x += ds * cosf(yaw);
  y += ds * sinf(yaw);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "stop" || line == "s") {
      set_cmd_vel(0, 0);
      Serial.println("ok stop");
    } else if (line.startsWith("v ")) {
      float lin = 0, ang = 0;
      if (sscanf(line.c_str() + 2, "%f %f", &lin, &ang) == 2) {
        set_cmd_vel(lin, ang);
        Serial.printf("ok v %.3f %.3f\n", lin, ang);
      }
    } else if (line == "zero") {
      x = y = yaw = 0;
      Serial.println("ok zero");
    }
  }

  uint32_t now = millis();
  if (now - last_odom_ms >= 50) {  // 20 Hz telemetry
    last_odom_ms = now;
    integrate_odom();
    float us = read_ultrasonic_m();
    int ir_l = digitalRead(PIN_IR_L) == LOW ? 1 : 0;  // typical: LOW = obstacle
    int ir_r = digitalRead(PIN_IR_R) == LOW ? 1 : 0;
    Serial.printf("st x=%.3f y=%.3f yaw=%.3f tl=%ld tr=%ld us=%.3f irl=%d irr=%d\n",
                  x, y, yaw, (long)ticks_l, (long)ticks_r, us, ir_l, ir_r);
  }
}
