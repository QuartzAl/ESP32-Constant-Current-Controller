// ================================================================= //
//      ESP32/ESP8266 Constant Current Source Firmware (v12)       //
// ================================================================= //
//
// This firmware is compatible with both ESP32 and ESP8266.
//
// v12 Changes:
// - Standardized PID and DAC output to an 8-bit range (0-255) for
//   both ESP32 and ESP8266, relying on a user-provided dacWrite
//   wrapper in util.h for ESP8266 compatibility.
// - Removed platform-specific preprocessor blocks for output logic.

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #define WebServer ESP8266WebServer // Alias for WebServer
#else // ESP32
  #include <WiFi.h>
  #include <WebServer.h>
#endif

#include <WiFiManager.h>
#include <Wire.h>
#include <INA219.h>
#include <PID_v1.h>
#include "config.h"
#include "index.h"
#include "util.h"


// --- INA219 Sensor ---
INA219 ina219(INA219_ADDRESS);

// --- PID Controller ---
double Setpoint, Input, Output;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Web Server ---
WebServer server(80);

// --- Global Variables ---
float busVoltage_V = 0;
float current_mA = 0;
double targetCurrent_mA = 100.0;
double maxCurrentLimit_mA = 500.0;


// --- Handler Functions for WebServer ---
void handleRoot() { server.send_P(200, "text/html", index_html); }

void handleData() {
    String jsonData = "{";
    jsonData += "\"voltage\":" + String(busVoltage_V, 2);
    jsonData += ", \"current\":" + String(current_mA, 2);
    jsonData += ", \"setpoint\":" + String(targetCurrent_mA, 2);
    jsonData += ", \"kp\":" + String(Kp);
    jsonData += ", \"ki\":" + String(Ki);
    jsonData += ", \"kd\":" + String(Kd);
    jsonData += ", \"max_limit\":" + String(maxCurrentLimit_mA);
    jsonData += "}";
    server.send(200, "application/json", jsonData);
}

void handleSet() {
  if (server.hasArg("current")) {
    double reqCurrent = server.arg("current").toDouble();
    targetCurrent_mA = min(reqCurrent, maxCurrentLimit_mA);
    Setpoint = targetCurrent_mA;
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Bad Request"); }
}

void handleSetPid() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    Kp = server.arg("kp").toDouble();
    Ki = server.arg("ki").toDouble();
    Kd = server.arg("kd").toDouble();
    myPID.SetTunings(Kp, Ki, Kd);
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Bad Request"); }
}

void handleSetAdvanced() {
    if (server.hasArg("max")) {
        maxCurrentLimit_mA = server.arg("max").toDouble();
        ina219.setMaxCurrentShunt(maxCurrentLimit_mA / 1000.0, SHUNT_RESISTOR_OHMS);
        if (targetCurrent_mA > maxCurrentLimit_mA) {
            targetCurrent_mA = maxCurrentLimit_mA;
            Setpoint = targetCurrent_mA;
        }
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

// --- Unified output function ---
void setOutputLevel(double pidOutput) {
  // This function now relies on the user-provided dacWrite wrapper in util.h
  // to handle the platform-specific output (DAC for ESP32, PWM for ESP8266).
  int dacValue = constrain((int)pidOutput, 1, 255);
  dacWrite(DAC_PIN, dacValue);
}


void setup() {
  Serial.begin(115200);
  #ifdef ESP8266
    // For ESP8266, the dacWrite wrapper in util.h handles analogWrite setup.
    // If specific setup like pinMode is needed, it should be in the wrapper.
  #endif
  Wire.begin();

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  if (!ina219.setMaxCurrentShunt(maxCurrentLimit_mA / 1000.0, SHUNT_RESISTOR_OHMS)) {
    Serial.println("INA219 calibration failed.");
    while (1) { delay(10); }
  }
  Serial.println("INA219 calibrated successfully.");

  setOutputLevel(0);

  WiFiManager wm;
  if (!wm.autoConnect("ESP-CurrentSource")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData); // New endpoint for data
  server.on("/set", HTTP_GET, handleSet);
  server.on("/setpid", HTTP_GET, handleSetPid);
  server.on("/setadvanced", HTTP_GET, handleSetAdvanced);
  
  server.begin();
  Serial.println("HTTP server started");

  Setpoint = targetCurrent_mA;
  myPID.SetMode(AUTOMATIC);
  // Set PID output limits to a standard 8-bit range for both platforms.
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  server.handleClient();

  busVoltage_V = ina219.getBusVoltage();
  current_mA = ina219.getCurrent_mA();

  if (busVoltage_V >= MAXIMUM_BUS_VOLTAGE_INA219 && targetCurrent_mA > current_mA) {
    // Safety override is now platform-agnostic.
    dacWrite(DAC_PIN, DAC_SAFETY_VALUE);
  } else {
    Input = current_mA;
    myPID.Compute();
    setOutputLevel(Output);
  }
}
