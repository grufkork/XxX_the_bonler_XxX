#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <Wire.h>

#include <ESPAsyncWebServer.h>

#include <VescUart.h>

#include "eigenModel.h"
#include "functions.h"

// En motor väger 2.9kg
// Hela väger 10.69kg

const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static AsyncWebServer server(80);
static AsyncWebSocket webSocket("/ws");

VescUart motorL;
VescUart motorR;

long TICKS_PER_REV = 90;
float WHEEL_RADIUS = 0.085f; // Meters
float WHEEL_CIRCUMFERENCE = 2.0f * PI * WHEEL_RADIUS;

// put function declarations here:
void ReadMPU();

float boning_constant = 0.8f;
float r_coeff = 1.6f;
float angle_offset = 0.07f;


#define LQR_MSG 0x01
#define BONING_MSG 0x02
#define R_COEFF_MSG 0x03

void setup() {
  Model.K_lqr <<
    -1.39734, -13.3706, -1.48825, -0.981489, -0.551298,
    -1.39734, -13.3706, -1.48825, -0.981489, -0.551298;


  Serial.begin(115200);
  Serial2.begin(115200);
  // Serial.println("Starting");
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  while(!Serial) {;}
  while(!Serial2){;}
  
  motorL.setSerialPort(&Serial);
  motorR.setSerialPort(&Serial2);
    

  WiFi.mode(WIFI_AP);
  // Serial.println("Starting AP");
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  WiFi.softAP("pro-bono", "ihardlyknowher");
  // Serial.println("Is ok?");
  

  
  
  webSocket.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)len;

    if (type == WS_EVT_CONNECT) {
      client->setCloseClientOnQueueFull(false);
    } else if (type == WS_EVT_DISCONNECT) {

    } else if (type == WS_EVT_ERROR) {

    } else if (type == WS_EVT_PONG) {

    } else if (type == WS_EVT_DATA) {
      switch(data[0]) {
        case LQR_MSG:
          if (len == 1 + 10 * sizeof(float)) {
            memcpy(Model.K_lqr.data(), data + 1, 10 * sizeof(float));
          }
          break;
        case BONING_MSG:
          if (len == 1 + sizeof(float)) {
            boning_constant = ((float*) (data + 1))[0];
          }
          break;
        case R_COEFF_MSG:
          if (len == 1 + sizeof(float)) {
            r_coeff = ((float*) (data + 1))[0];
          }
          break;
      }
    }
  });

  /*server.addHandler(&ws).addMiddleware([](AsyncWebServerRequest *request, ArMiddlewareNext next) {
    if (ws.count() >= 1) {
      request->send(503, "text/plain", "Server is busy");
    } else {
      next();
    }
  });*/

  server.addHandler(&webSocket);

  server.begin();
  

  /// Model.ad FIXME


}

float t = 0.0f;

float avgWheelSpeedR = 0.0f;
float avgWheelSpeedL = 0.0f;

float rWheelPosLast = 0.0f;
float lWheelPosLast = 0.0f;

unsigned long lastMicros = 0;

float lastPos = 0.0f;

float lastAngle = 0.0f;


void loop() {
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;
  // motorL.setDuty(sin(t)* 1.0f);
  // t += 0.001;
  
  long motorLPos, motorRPos;

  // if ( motorL.getVescValues() ) {
  //   motorLPos = motorL.data.tachometer;
  // }
  // if ( motorR.getVescValues() ) {
  //   motorRPos = motorR.data.tachometer;
  // }

  float rWheelPos = (float)motorRPos / TICKS_PER_REV * WHEEL_CIRCUMFERENCE;
  float lWheelPos = (float)motorLPos / TICKS_PER_REV * WHEEL_CIRCUMFERENCE;
  
  float rWheelSpeed = (rWheelPos - rWheelPosLast) / dt;
  float lWheelSpeed = (lWheelPos - lWheelPosLast) / dt;

  rWheelPosLast = rWheelPos;
  lWheelPosLast = lWheelPos;

  avgWheelSpeedL = 0.9f * avgWheelSpeedL + 0.1f * lWheelSpeed;
  avgWheelSpeedR = 0.9f * avgWheelSpeedR + 0.1f * rWheelSpeed;
  
  float pos = (rWheelPos + lWheelPos) / 2.0f;
  float velocity = (rWheelSpeed + lWheelSpeed) / 2.0f;
  lastPos = pos;

  ReadMPU();
  
  float accAngle = atan2f((float)AcY, (float)AcZ) - angle_offset;

  float gyroSpeed = (float)GyX / 131.0f * DEG_TO_RAD;
  
  float gamma = 0.995f;

  float angle = gamma * (lastAngle + gyroSpeed * dt) + (1.0f - gamma) * accAngle;
  lastAngle = angle;



  float angVel = (angle - lastAngle) / dt;
  lastAngle = angle;

  
  VectorXf state(5);
  state << pos, angle, velocity, angVel, 0.0f;
  Vector2f tau = -Model.K_lqr * state;

  float tau_refL = tau(0);
  float tau_refR = tau(1);
  
  // Serial.println(angle);

  Model.u_prev << tau_refL, tau_refR;

  motorL.setCurrent(-tau_refL * boning_constant);
  motorR.setCurrent(tau_refR * boning_constant * r_coeff);

  // return;
  // Serial.println(angle * 100.0f);

  // Serial.println(AcX);
  // Serial.println(AcY);
  // Serial.println(AcZ);
  // Serial.println(GyX);
  // Serial.println(GyY);
  // Serial.println(GyZ);
  
  // Serial.println();
  // Serial.println();
  
  // delay(10);
  
}

void ReadMPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  Tmp = (Wire.read() << 8) | Wire.read();
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();
}