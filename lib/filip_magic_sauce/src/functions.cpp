#include "functions.h"

Adafruit_MPU6050 mpu;
TwoWire Wire2 = TwoWire(1);

AS5600L encoderL(Wire, 0x40);
AS5600L encoderR(Wire2, 0x40);

LPfilter::LPfilter(float filterWeight, float initialValue)
    : weight(filterWeight), filteredValue(initialValue) {}

float LPfilter::update(float input) {
    filteredValue = (1 - weight) * input + weight * filteredValue;
    return filteredValue;
}

PwmMotor::PwmMotor(int pin_IN1, int pin_IN2,
                   int pwm_channel1, int pwm_channel2,
                   int pwm_freq, int pwm_resolution,
                   bool reverse_direction) {
    IN1 = pin_IN1;
    IN2 = pin_IN2;
    freq = pwm_freq;
    resolution = pwm_resolution;
    channel1 = pwm_channel1;
    channel2 = pwm_channel2;
    reverse = reverse_direction;
}

void PwmMotor::motorInit() {
    ledcSetup(channel1, freq, resolution);
    ledcSetup(channel2, freq, resolution);
    ledcAttachPin(IN1, channel1);
    ledcAttachPin(IN2, channel2);
}

void PwmMotor::calculateParams(float U_rated, float w_noload,
                               float tau_stall, float i_stall,
                               float R_mot) {
    R = R_mot;
    ke = U_rated / w_noload;
    kt = tau_stall / i_stall;
}

void PwmMotor::setRegulatorParams(float Kp_T_reg, float Ki_T_reg, float Kd_T_reg,
                                  float Kp_w_reg, float Ki_w_reg, float Kd_w_reg) {
    Kp_T = Kp_T_reg;
    Ki_T = Ki_T_reg;
    Kd_T = Kd_T_reg;
    Kp_w = Kp_w_reg;
    Ki_w = Ki_w_reg;
    Kd_w = Kd_w_reg;
}

void PwmMotor::motorWriteTorque(float tau_ref, float Vin,
                                float i_mot, float phi, float dt) {
    static float int_i_err = 0;

    float i_ref = tau_ref / kt;
    float V_drop = R * i_ref + ke * phi;
    float i_err = i_ref - i_mot;

    int_i_err += i_err * dt;
    int_i_err = constrain(int_i_err, -intLimit, intLimit);

    float V_corr = Kp_T * i_err + Ki_T * int_i_err - Kd_T * phi;
    float V_cmd = V_drop + V_corr;

    float dutycycle = V_cmd / Vin;
    dutycycle = constrain(dutycycle, -1.0f, 1.0f);

    motorWrite(dutycycle);
}

void PwmMotor::motorWriteSpeed(float phi_ref, float Vin,
                               float i_mot, float phi, float dt) {
    static float int_w_err = 0;
    static float prev_w_err = 0;
    static float alpha = 0.95;

    float w_err = alpha * prev_w_err + (1 - alpha) * (phi_ref - phi);
    int_w_err += w_err * dt;
    int_w_err = constrain(int_w_err, -intLimit, intLimit);

    float tau_ref = Kp_w * w_err
                  + Ki_w * int_w_err
                  - Kd_w * (w_err - prev_w_err) / dt;

    prev_w_err = w_err;
    motorWriteTorque(tau_ref, Vin, i_mot, phi, dt);
}

void PwmMotor::motorWrite(float dutycycle) {
    bool forward = dutycycle >= 0;
    int duty = round(abs(dutycycle) * maxDuty);

    if (forward ^ reverse) {
        ledcWrite(channel1, duty);
        ledcWrite(channel2, 0);
    } else {
        ledcWrite(channel1, 0);
        ledcWrite(channel2, duty);
    }
}

void PwmMotor::motorBrake(float dutycycle, bool forward) {
    int duty = round(dutycycle * maxDuty);
    ledcWrite(channel1, duty);
    ledcWrite(channel2, duty);
}

void PwmMotor::motorDisable() {
    ledcWrite(channel1, 0);
    ledcWrite(channel2, 0);
}

// ================= AS5600L =================
AS5600L::AS5600L(TwoWire& wirePort, uint8_t i2c_address) {
    i2c = &wirePort;
    address = i2c_address;
}

void AS5600L::readAngle(float dt) {
    i2c->beginTransmission(address);
    i2c->write(ANGLE_REG_MSB);
    i2c->endTransmission(false);
    i2c->requestFrom(address, (uint8_t)2);

    // Do not skip updates silently
    if (i2c->available() < 2) return;

    uint8_t highByte = i2c->read();
    uint8_t lowByte  = i2c->read();
    currentPos = (lowByte << 8) | highByte;

    if (!init) {
        lastPos = currentPos;
        angle = 0.0f;
        angVel = 0.0f;
        init = true;
        return;
    }

    int delta = currentPos - lastPos;
    if (delta > 2048)      delta -= 4096;
    else if (delta < -2048) delta += 4096;

    lastPos = currentPos;

    // Update angle
    float dtheta = delta * 2.0f * PI / 4096.0f;
    angle += dtheta;

    // Compute velocity directly from delta (robust to missed samples)
    angVel = dtheta / dt;
}

void AS5600L::reset() {
    init = false;
    angle = 0;
    angVel = 0;
}

void resetEncoders() {
    encoderL.reset();
    encoderR.reset();
}

// ================= Functions =================
VectorXf readEncoders(float dt) {
    float r = 0.025;

    encoderL.readAngle(dt);
    encoderR.readAngle(dt);

    float velL = encoderL.angVel * r;
    float velR = encoderR.angVel * r;

    VectorXf x(2);
    x << (encoderR.angle + encoderL.angle) * r / 2,
         (velR + velL) / 2;
    return x;
}

VectorXf readMPU(float dt) {
    static VectorXf imu_vals = VectorXf::Zero(4);
    static float gyro_ang_roll = 0;
    static float gyro_ang_yaw = 0;

    sensors_event_t a, G, temp;
    mpu.getEvent(&a, &G, &temp);

    float gravity_x = -a.acceleration.x + 0.2;
    float gravity_z = -a.acceleration.z + 0.0905;
    float ang_vel_yaw = -G.gyro.z + 0.0301;
    float trig_ang = atan2(gravity_x, gravity_z);
    float ang_vel_roll = -G.gyro.y;

    float gamma = 0.995;
    gyro_ang_roll = gamma * (gyro_ang_roll + dt * ang_vel_roll) + (1 - gamma) * trig_ang;
    gyro_ang_yaw += ang_vel_yaw * dt;

    imu_vals << -gyro_ang_roll, -ang_vel_roll, gyro_ang_yaw, ang_vel_yaw;
    return imu_vals;
}

LedcServo::LedcServo(uint8_t pin, uint8_t channel, uint32_t freq,
                     uint8_t resolution, uint16_t minUs, uint16_t maxUs)
{
    this->pin = pin;
    this->channel = channel;
    this->freq = freq;
    this->resolution = resolution;
    this->minUs = minUs;
    this->maxUs = maxUs;
}

void LedcServo::begin()
{
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);

    uint32_t maxDutyCount = (1UL << resolution) - 1;
    float periodUs = 1000000 / freq;

    minDuty = (uint32_t)((minUs / periodUs) * maxDutyCount);
    maxDuty = (uint32_t)((maxUs / periodUs) * maxDutyCount);
}

void LedcServo::write(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t duty =
        minDuty + (uint32_t)((maxDuty - minDuty) * (angle / 180.0f));

    ledcWrite(channel, duty);
}
