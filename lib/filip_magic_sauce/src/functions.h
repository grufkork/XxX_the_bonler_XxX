#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Adafruit_MPU6050.h>
#include <ArduinoEigenDense.h>
#include <Wire.h>

using namespace std;
using namespace Eigen;

// External globals
extern Adafruit_MPU6050 mpu;
extern TwoWire Wire2;

class LPfilter {
private:
    float weight;
    float filteredValue;

public:
    LPfilter(float filterWeight, float initialValue);
    float update(float input);
};

class PwmMotor {
private:
    int IN1 = 0;
    int IN2 = 0;
    int freq = 200;
    int channel1 = 0;
    int channel2 = 0;
    int resolution = 10;
    int maxDuty = (1 << resolution) - 1;
    bool reverse = false;

    // motor parameters
    float ke = 0.01;
    float kt = 0.01;
    float R = 3;

    // control parameters
    float Kp_T = 10;
    float Ki_T = 0;
    float Kd_T = 0;
    float Kp_w = 0.1;
    float Ki_w = 0;
    float Kd_w = 0;
    float intLimit = 1;

public:
    PwmMotor(int pin_IN1, int pin_IN2,
             int pwm_channel1, int pwm_channel2,
             int pwm_freq, int pwm_resolution,
             bool reverse_direction);

    void motorInit();
    void calculateParams(float U_rated, float w_noload,
                         float tau_stall, float i_stall,
                         float R_mot);
    void setRegulatorParams(float Kp_T_reg, float Ki_T_reg, float Kd_T_reg,
                            float Kp_w_reg, float Ki_w_reg, float Kd_w_reg);

    void motorWriteTorque(float tau_ref, float Vin,
                          float i_mot, float phi, float dt);
    void motorWriteSpeed(float phi_ref, float Vin,
                         float i_mot, float phi, float dt);
    void motorWrite(float dutycycle);
    void motorBrake(float dutycycle, bool forward);
    void motorDisable();
};

class AS5600L {
private:
    uint8_t address = 0x40;
    uint8_t ANGLE_REG_LSB = 0x0C;
    uint8_t ANGLE_REG_MSB = 0x0D;

    bool init = false;
    float lastAngle = 0;
    float lastAngVel = 0;
    uint16_t initialPos = 0;
    uint16_t currentPos = 0;
    uint16_t lastPos = 0;
    int delta = 0;

    TwoWire* i2c;

public:
    float angle = 0;
    float angVel = 0;

    AS5600L(TwoWire& wirePort, uint8_t i2c_address = 0x40);
    void readAngle(float dt);
    void reset();
};

// External encoders
extern AS5600L encoderL;
extern AS5600L encoderR;

// Functions
VectorXf readEncoders(float dt);
VectorXf readMPU(float dt);
void resetEncoders();

class LedcServo
{
public:
    LedcServo(uint8_t pin,
              uint8_t channel,
              uint32_t freq = 50,
              uint8_t resolution = 14,
              uint16_t minUs = 500,
              uint16_t maxUs = 2500);

    void begin();
    void write(float angle);

private:
    uint8_t pin;
    uint8_t channel;
    uint32_t freq;
    uint8_t resolution;

    uint16_t minUs;
    uint16_t maxUs;

    uint32_t minDuty;
    uint32_t maxDuty;
};

#endif // FUNCTIONS_H
