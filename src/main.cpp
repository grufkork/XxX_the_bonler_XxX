#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <Wire.h>

#include <ESPAsyncWebServer.h>

#include <VescUart.h>

#include "eigenModel.h"
#include "functions.h"

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

float constant_boning = 0.8f;
float r_constant = 1.6f;
float angle_offset = 0.07f;

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
      webSocket.textAll("new client connected");
      client->setCloseClientOnQueueFull(false);

    } else if (type == WS_EVT_DISCONNECT) {
      webSocket.textAll("client disconnected");

    } else if (type == WS_EVT_ERROR) {

    } else if (type == WS_EVT_PONG) {

    } else if (type == WS_EVT_DATA) {
      constant_boning = ((float)data[0]) / 255.0f;
      // motorL.setCurrent(((float)data[0]) / 255.0f * 3.0f);
      // motorR.setCurrent(-((float)data[1]) / 255.0f * 3.0f);
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

  motorL.setCurrent(-tau_refL * constant_boning);
  motorR.setCurrent(tau_refR * constant_boning * r_constant);

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