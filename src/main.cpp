#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <Wire.h>
#include "littleFS.h"
#include <ESPAsyncWebServer.h>

#include <VescUart.h>

#include "eigenModel.h"

// En motor väger 2.9kg
// Hela väger 10.69kg

const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static AsyncWebServer server(80);
static AsyncWebSocket webSocket("/ws");

VescUart motorL;
VescUart motorR;

float TICKS_PER_REV = 90.0f;
float WHEEL_RADIUS = 0.085f; // Meters
float WHEEL_CIRCUMFERENCE = 2.0f * PI * WHEEL_RADIUS;

// put function declarations here:
void ReadMPU();

float current_gain = 1.1f;
long tachometer_sign = 1;
float accelerometer_sign = 1.0f;
float r_coeff = 1.4f;
float angle_offset = -0.13f; // Enter the value when bonler is straight

const float MAX_ANGLE = 20.0f * DEG_TO_RAD;


#define UNUSED0 0x01
#define UNUSED1 0x02
#define UNUSED2 0x03
#define LQR_PARAMS_MSG 0x04
#define ON_MSG 0x05
#define OFF_MSG 0x06
#define CONTROL_INPUT_MSG 0x07

volatile bool running = false;

volatile float throttle = 0.0f;
volatile float steering = 0.0f;

void Stop();
void Reset();

float telemetry[1024] = {};
int telemetry_index = 0;

void add_telemetry(float value) {
  telemetry[telemetry_index] = value;
  telemetry_index += 1;
}

AsyncWebSocketClient* latest_client = nullptr;
void log_message(const char* message) {
  // Serial.println(message);
  if (latest_client != nullptr) {
    webSocket.text(latest_client->id(), message);
  }
}

float M = 5.4;            // Bonler mass
float m = 2.9;          // Wheel mass
float Jb = 1.62;  // Body inertia
float Jw = 0.02;  // Wheel inertia
float g = 9.82;
float l = 0.207;         // body COM length from wheel axis
float r = 0.085;        // Wheel radius


volatile bool runningInitialised = false;

float Kp = 2.0f;
float Kw = 0.8f;

void solve_lqr() {

  Model.Qu << 0.1, 0,
              0, 0.1; // Should be the same

  log_message("Solving LQR");
  Model.Ac << 0, 0,                                                                                                                                       1, 0,
              0, 0,                                                                                                                                       0, 1,
              0, -(pow(M,2)*g*pow(l,2)*pow(r,2))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),                 0, 0,
              0, (M*l*(2*Jw*g + 2*g*m*pow(r,2) + M*g*pow(r,2)))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),  0, 0;
  log_message("Ac set");

  Model.Bc << 0, 0,
              0, 0,
              (r*(M*pow(l,2) + Jb))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),  (r*(M*pow(l,2) + Jb))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),
              -(M*l*r)/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),               -(M*l*r)/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2));
  
  Model.C << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 0, 1;

  Model.Q << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

  Model.R << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;

  log_message("Bc set");
  
  Model.discretize_state_matricies(); // creates Ad and Bd
  log_message("State matricies discretized");
  Model.solveRicatti(); // Creates K_lqr if solution is found
  log_message("LQR solved");
}



void setup() {
  // x, theta, x_dot, theta_dot,
  Model.Qx << 5, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  WiFi.mode(WIFI_AP);
  // Serial.println("Starting AP");
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  WiFi.softAP("pro-bono", "ihardlyknowher");
    if(!LittleFS.begin(true)){
    Serial.println("File mount failed");
    return;
  }
  

  // Serial.println("Is ok?");
  // Model.K_lqr <<
  //   -1.39734, -13.3706, -1.48825, -0.981489, -0.551298,
  //   -1.39734, -13.3706, -1.48825, -0.981489, -0.551298;


  Serial.begin(115200);
  Serial2.begin(115200, 134217756UL, 16, 17);
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
    

  solve_lqr();
  
  

  
  
  webSocket.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)len;

    if (type == WS_EVT_CONNECT) {
      client->setCloseClientOnQueueFull(false);
      latest_client = client;
    } else if (type == WS_EVT_DISCONNECT) {
      if (latest_client == client) {
        latest_client = nullptr;
      }
    } else if (type == WS_EVT_ERROR) {

    } else if (type == WS_EVT_PONG) {

    } else if (type == WS_EVT_DATA) {
      switch(data[0]) {
        case LQR_PARAMS_MSG:
          if (len == 1 + 17 * sizeof(float)) {
            float* params = (float*)(data + 1);
            M = params[0];
            m = params[1];
            Jb = params[2];
            Jw = params[3];
            g = params[4];
            l = params[5];
            r = params[6];
            current_gain = params[7];
            tachometer_sign = static_cast<long>(params[8]);
            accelerometer_sign = params[9];
            r_coeff = params[10];
            Model.Qx(0,0) = params[11];
            Model.Qx(1,1) = params[12];
            Model.Qx(2,2) = params[13];
            Model.Qx(3,3) = params[14];
            Kp = params[15];
            Kw = params[16];
            
            log_message("Ack, solving LQR");
            solve_lqr();
          }
          break;
        case ON_MSG:
          Model.resetKalman();
          runningInitialised = false;
          Reset();
          log_message("Turned on");
          running = true;
          break;
        case OFF_MSG:
          Stop();
          break;
        case CONTROL_INPUT_MSG:
          if(len == 1 + 2*sizeof(float)){
            float* in = (float*)(data + 1);
            throttle = in[0];
            steering = in[1];
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
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  
  server.begin();
}

void Stop(){
  running = false;
  motorL.setCurrent(0.0f);
  motorR.setCurrent(0.0f);
  log_message("Turned off");
  Reset();
}

void Reset(){
  runningInitialised = false;
  Model.x_ref = Model.x_ref.setZero();
  log_message("Reset");
}

float t = 0.0f;

float avgWheelSpeedR = 0.0f;
float avgWheelSpeedL = 0.0f;

float rWheelPosLast = 0.0f;
float lWheelPosLast = 0.0f;

unsigned long lastMicros = 0;

float lastPos = 0.0f;

float lastAngle = 0.0f;

float yawAngle = 0.0f;

unsigned long stepPeriodMicros = 10 * 1000;

long initialMotorLPos, initialMotorRPos = 0;

int loopn = 0;

int num_telemetry_requested = 0;

unsigned long timeOfLastControlInput = 0;

void loop() {
  unsigned long currentMicros = micros();
  unsigned long micros_at_next_step = currentMicros + stepPeriodMicros;
  float dt = (currentMicros -lastMicros) / 1000000.0f;
  lastMicros = currentMicros;
  // motorL.setDuty(sin(t)* 1.0f);
  // t += 0.001;
  
  if (!running) {
    return;
  }
  
  bool justStarted = !runningInitialised;
  if (justStarted) {
    runningInitialised = true;
    loopn = 0;
    avgWheelSpeedL = 0.0f;
    avgWheelSpeedR = 0.0f;
    yawAngle = 0.0f;
  }
  
  // Serial.print(dt * 1000.0f);
  // Serial.print("Time step: ");
  

  // if ( motorL.getVescValues() ) {
  //   motorLPos = motorL.data.tachometer;
  // }
  
  if (justStarted){
    log_message("Clearing UART queue");
    int queuelen = 0;
    while(Serial.available()){
      Serial.read();
      queuelen ++;
    }
    log_message(("Cleared " + String(queuelen) + " bytes in L").c_str());
    queuelen = 0;
    while(Serial2.available()){
      Serial2.read();
      queuelen ++;
    }
    log_message(("Cleared " + String(queuelen) + " bytes in R").c_str());
    motorR.requestVescValues();
    motorL.requestVescValues();
    num_telemetry_requested = 1;
    log_message("Cleared UART");
  }

  long motorLPos, motorRPos = 0;

  if (num_telemetry_requested > 0){
    if(!motorR.readVescValues()){
      log_message("Failed to read motor R values");
      Stop();
      return;
    }
    if(!motorL.readVescValues()){
      log_message("Failed to read motor L values");
      Stop();
      return;
    }
    num_telemetry_requested -= 1;
    motorLPos = motorL.data.tachometer * tachometer_sign;
    motorRPos = motorR.data.tachometer * tachometer_sign;
    if (justStarted){
      initialMotorLPos = motorLPos;
      initialMotorRPos = motorRPos;
    }
    motorLPos -= initialMotorLPos;
    motorRPos -= initialMotorRPos;
    // log_message(("L: " + String(motorLPos) + ", R: " + String(motorRPos)).c_str());
  }else{
    log_message("No message available for reading!!");
    Stop();
  }
  if (num_telemetry_requested <= 0){
    motorR.requestVescValues();
    motorL.requestVescValues();
    num_telemetry_requested ++;
  }else{
    log_message("Message already requested");
  }


  float rWheelPos = static_cast<float>(motorRPos) / TICKS_PER_REV * WHEEL_CIRCUMFERENCE;
  float lWheelPos = static_cast<float>(motorLPos) / TICKS_PER_REV * WHEEL_CIRCUMFERENCE;
  
  if (justStarted) {
    rWheelPosLast = rWheelPos;
    lWheelPosLast = lWheelPos;
  }
  
  float rWheelSpeed = (rWheelPos - rWheelPosLast) / dt;
  float lWheelSpeed = (lWheelPos - lWheelPosLast) / dt;

  rWheelPosLast = rWheelPos;
  lWheelPosLast = lWheelPos;

  avgWheelSpeedL = 0.5f * avgWheelSpeedL + 0.5f * lWheelSpeed;
  avgWheelSpeedR = 0.5f * avgWheelSpeedR + 0.5f * rWheelSpeed;

  
  float pos = (rWheelPos + lWheelPos) / 2.0f;
  float velocity = (avgWheelSpeedR + avgWheelSpeedL) / 2.0f;
  
  if(abs(velocity) > 1.0f) {
    Stop();
    log_message("!!! Velocity too high, stopping !!!");
    return;
  }
  
  ReadMPU();
  
  float accAngle = atan2f((float)AcY, (float)AcZ) - angle_offset;

  float gyroSpeed = (float)GyX / 131.0f * DEG_TO_RAD;
  
  float gamma = 0.995f;
  
  float angle = (gamma * (lastAngle + gyroSpeed * dt) + (1.0f - gamma) * accAngle)*accelerometer_sign;
  if(abs(angle) > MAX_ANGLE) {
    Stop();
    log_message("!!! Angle too steep, stopping !!!");
    return;
  }

  // Yaw controller
  float yawRate = (float)GyZ / 131.0f * DEG_TO_RAD;
  yawAngle += yawRate * dt;

  yawAngle += steering * dt;
  Model.x_ref[0] += throttle * dt;

  float yawControl = Kp * yawAngle + Kw * yawRate;
  
  if (justStarted){
    angle = 0.0f;
  }

  // float angVel = (angle - lastAngle) / dt;
  float angVel = gyroSpeed;
  lastAngle = angle;

  // Vector<float, Model.n> estimatedState = Model.kalmanFilter(measurement);
  Vector<float, Model.n> estimatedState = Vector4f(pos, angle, velocity, angVel) - Model.x_ref;
  
  Vector2f tau = -Model.K_lqr * estimatedState;

  float tau_refL = tau(0) + yawControl;
  float tau_refR = tau(1) - yawControl;
  

  Model.u_prev << tau_refL, tau_refR;// TODO Reset this as well when Kalman is used

  motorL.setCurrent(tau_refL * current_gain);
  motorR.setCurrent(tau_refR * current_gain * r_coeff);
  
  add_telemetry(pos);
  add_telemetry(angle);
  add_telemetry(velocity);
  add_telemetry(angVel);
  add_telemetry(tau_refL);
  add_telemetry(tau_refR);

  if(telemetry_index >= 500){
    webSocket.binary(latest_client->id(), (uint8_t*)telemetry, telemetry_index * sizeof(float));
    telemetry_index = 0;
  }
  
  if (loopn % 25 == 0) {
    log_message(&("Pos: " + String(pos) + ", Ang: " + String(angle) + ", Vel: " + String(velocity) + ", Angvel: " + String(angVel))[0]);
    log_message(&("Tau: " + String(tau_refL) + ", " + String(tau_refR))[0]);
    // log_message(&("Time: " + String(dt*1000.0f))[0]);
  }
  loopn++;
  
  while (micros() < micros_at_next_step) {}
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