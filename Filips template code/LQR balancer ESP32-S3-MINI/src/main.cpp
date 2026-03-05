#include <Arduino.h>
#include <Wire.h>

#include "functions.h"
#include "eigenModel.h"
#include "printLinalg.h"

#define SCL_PIN 2
#define SDA_PIN 3

#define SCL2_PIN 37
#define SDA2_PIN 38

#define INP_VOLTAGE_SENSE 10
#define CURRENT_SEN_M1 5
#define CURRENT_SEN_M2 4
float CURRENT_SEN_M1_OFFSET = 1977.6;
float CURRENT_SEN_M2_OFFSET = 2218.6;

// Motor PWM
#define PWM1_M1 13
#define PWM2_M1 14
#define PWM1_M2 15
#define PWM2_M2 16

// Servo PWM
#define SERVO1 9
#define SERVO2 8
#define SERVO3 7
#define SERVO4 6

LPfilter voltageFilter(0.98, 7.4);
LPfilter currentM1filter(0, 0);
LPfilter currentM2filter(0, 0);

LPfilter yawRateFilter(0.9, 0);

int freq = 200;
int resolution = 10; // Bits
int channel1_M1 = 1; int channel2_M1 = 2;
int channel1_M2 = 3; int channel2_M2 = 4;
int servo_channel = 5;

PwmMotor Motor1(PWM1_M1, PWM2_M1, channel1_M1, channel2_M1, freq, resolution, false); // Left
PwmMotor Motor2(PWM1_M2, PWM2_M2, channel1_M2, channel2_M2, freq, resolution, true);  // Right
LedcServo servo(SERVO1, servo_channel);

void setup(){
  /////////////////////////////////////////////////////////
  delay(2000);
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire2.begin(SDA2_PIN, SCL2_PIN);

  Wire.setClock(400000);
  Wire2.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

  pinMode(INP_VOLTAGE_SENSE, INPUT);
  pinMode(CURRENT_SEN_M1, INPUT);
  pinMode(CURRENT_SEN_M2, INPUT);

  CURRENT_SEN_M1_OFFSET = analogRead(CURRENT_SEN_M1);
  CURRENT_SEN_M2_OFFSET = analogRead(CURRENT_SEN_M2);

  pinMode(PWM1_M1, OUTPUT); pinMode(PWM2_M1, OUTPUT);
  pinMode(PWM1_M2, OUTPUT); pinMode(PWM2_M2, OUTPUT);

  Motor1.calculateParams(7.45, 6500*PI/30, 0.015, 1.05, 3);
  Motor2.calculateParams(7.45, 6500*PI/30, 0.015, 1.05, 4);

  Motor1.setRegulatorParams(8, 0, 0.2, 0.05, 0, 0);
  Motor2.setRegulatorParams(8, 0, 0.2, 0.05, 0, 0);

  Motor1.motorInit();
  Motor2.motorInit();

  servo.begin();

  /////////////////////////////////////////////////////////

  Model.Ad << 1, -0.000308052, 0.01, -1.02575e-06, 0,
              0, 1.01601, 0, 0.0100533, 0,
              0, -0.0617743, 1, -0.000308052, 0,
              0, 3.20995, 0, 1.01601, 0,
              0.01, 0, 0, 0, 1;

  Model.Bd << 0.0175714, 0.0175714,
              -0.539226, -0.539226,
              3.5198, 3.5198,
              -108.132, -108.132,
              0, 0;

  Model.Q << 0.004, 0, 0, 0, 0,
             0, 0.04, 0, 0, 0,
             0, 0, 0.004, 0, 0,
             0, 0, 0, 0.04, 0,
             0, 0, 0, 0, 0.0001;
  
             // Integrated/derived variables have higher variance.
  Model.R << 0.005, 0, 0, 0, 0,
             0, 0.05, 0, 0, 0,
             0, 0, 0.005, 0, 0,
             0, 0, 0, 0.05, 0,
             0, 0, 0, 0, 0;

  // LQR
  // pos, ang, vel, ang vel
  Model.x_ref << 0, 0, 0, 0, 0;

  // LQI demo mode
  Model.K_lqr.resize(2,5);
  Model.K_lqr << -0.0178365, -0.0401008, -0.0144751, -0.00529221, -0.00164394,
                 -0.0178365, -0.0401008, -0.0144751, -0.00529221, -0.00164394;

}

void loop() {
  ////////////////////////////////////////////////////////

  float vSense = analogRead(INP_VOLTAGE_SENSE);
  float voltage = voltageFilter.update(1.0*20.13*vSense/4096);

  float sensitivity = 0.090; // V/A
  float iSense_M1 = analogRead(CURRENT_SEN_M1);
  float current_M1 = currentM1filter.update(-3.3*(iSense_M1 - CURRENT_SEN_M1_OFFSET)/(4096*sensitivity));

  float iSense_M2 = analogRead(CURRENT_SEN_M2);
  float current_M2 = currentM2filter.update(3.3*(iSense_M2 - CURRENT_SEN_M2_OFFSET)/(4096*sensitivity));

  ////////////////////////////////////////////////////////

  unsigned static long lastTime = millis();
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= Model.Ts*1000) {

      float dt = (currentTime - lastTime)/1000.0f;
      lastTime = currentTime;

      static float t = 0;
      t += dt;

      // Drive with velocity
      //static float pos_ref = 0;
      //float vel_ref = 0.1;
      //pos_ref += dt*vel_ref; // 0.1 m/s

      // Oscillate
      //float osc_amplitude = 0.1;
      //float osc_freq = 0.25;
      //float pos_ref = osc_amplitude*sin(2*PI*osc_freq*t);
      //float vel_ref = osc_amplitude*cos(2*PI*osc_freq*t);

      // Model.x_ref << pos_ref, 0, vel_ref, 0, 0;

      static bool fell = false;
      static bool was_fell = false;
      static float fall_reset_timer = 0;

      VectorXf y_meas(5);
      Vector2f yaw_controller;
      Vector2f tau_ref_with_yaw;
      static float x_int = 0;

      // Read IMU and detect fallen robot
      VectorXf imu = readMPU(dt);       // angle and angular velocity (rad, rad/s)

      bool fell_now = (abs(imu(0)) >= 1);
      if (fell_now && !was_fell) {
          Model.resetKalman();
          resetEncoders();
          fall_reset_timer = 2.0f;
      }
      fell = fell_now;
      was_fell = fell_now;

      if (!fell) {

        VectorXf enc = readEncoders(dt);  // position and velocity (m, m/s)
        y_meas << enc(0), imu(0), enc(1), imu(1), 0;
        x_int += enc(0)*dt;
        y_meas(4) = x_int;
        
        // Estimate current states
        VectorXf x_est = Model.kalmanFilter(y_meas);

        // Servo
        float servo_offset = 10; // degrees
        float servo_angle = -y_meas(1)*180/PI;
        if (servo_angle >= 30) { servo_angle = 30; }
        if (servo_angle <= -30) { servo_angle = -30; }
        servo.write(90 + servo_angle + servo_offset);

        // Yaw control
        float Kp = 0.1;
        float Kw = 0.005;

        float yaw_angle = imu(2);
        float yaw_ang_vel = yawRateFilter.update(imu(3));
        float yaw_ref = 0;

        float yaw_error = yaw_angle - yaw_ref;

        float ctrl = Kp*yaw_error + Kw*yaw_ang_vel;
        if (ctrl >= 0.01) { ctrl = 0.01; }
        if (ctrl <= -0.01) { ctrl = -0.01; }
        yaw_controller << ctrl, -ctrl;

        // Controller
        VectorXf x_dev = x_est - Model.x_ref;
        Vector2f tau = - Model.K_lqr*x_dev;

        float tau_refL = tau(0);
        float tau_refR = tau(1);

        tau_ref_with_yaw(0) = tau_refL + yaw_controller(0);
        tau_ref_with_yaw(1) = tau_refR + yaw_controller(1);

        // Save current input for next iteration in Kalman filter
        Model.u_prev << tau_refL, tau_refR;
        
      }
      else {
        tau_ref_with_yaw.setZero();
      }

      // Motor driving and reseting
      if (fall_reset_timer > 0) {
        Motor1.motorDisable();
        Motor2.motorDisable();

        if (!fell) {
          fall_reset_timer -= dt;
        }
      }
      else {
        fall_reset_timer = 0;
        Motor1.motorWriteTorque(tau_ref_with_yaw(0), voltage, current_M1, encoderL.angVel, dt);
        Motor2.motorWriteTorque(tau_ref_with_yaw(1), voltage, current_M2, encoderR.angVel, dt);
      }
    }
}
