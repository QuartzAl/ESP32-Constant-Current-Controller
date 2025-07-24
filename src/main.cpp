// ================================================================= //
//      ESP32 Constant Current Source Controller Firmware (v7)     //
// ================================================================= //
//
// This firmware uses an ESP32 to create a constant current source.
// It features a PID controller, a web interface for control and
// monitoring, and uses Server-Sent Events for live data updates.
//
// Required Libraries:
// - WiFi: for network connection
// - ESPAsyncWebServer: for creating the web server
// - WiFiManager by tzapu: for easy WiFi configuration
// - INA219 by Rob Tillaart: for current and voltage sensing
// - PID by Brett Beauregard: for PID control loop

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <INA219.h>
#include <PID_v1.h>
#include "index.h"
#include "config.h"

#ifdef ESP8266
#include "util.h"
#endif

INA219 ina219(INA219_ADDRESS);

// --- PID Controller ---
double Setpoint, Input, Output; // PID variables
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;   // PID tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Web Server and SSE ---
AsyncWebServer server(80);
AsyncEventSource events("/events");

// --- Global Variables ---
float busVoltage_V = 0;
float current_mA = 0;
double targetCurrent_mA = 100.0; // Default target current in mA
double maxCurrentLimit_mA = 500.0; // Default max current limit
unsigned long sseInterval = 1000; // Default SSE update interval in ms

// --- Function to scale PID output to DAC value ---
void setBuckFeedback(double pidOutput) {
    int dacValue = constrain((int)pidOutput, 1, 255);
    dacWrite(DAC_PIN, dacValue);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Must be called before ina219.begin()

  // --- Initialize INA219 ---
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  // --- Calibrate INA219 ---
  if (!ina219.setMaxCurrentShunt(maxCurrentLimit_mA / 1000.0, SHUNT_RESISTOR_OHMS)) {
    Serial.println("INA219 calibration failed. Check connection and values.");
    while (1) { delay(10); }
  }
  Serial.println("INA219 calibrated successfully.");

  // --- Initialize DAC ---
  dacWrite(DAC_PIN, 1);

  WiFiManager wm;
  if (!wm.autoConnect("ESP32-CurrentSource")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // --- Web Server Routes ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/set", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("current")) {
        double reqCurrent = request->getParam("current")->value().toDouble();
        targetCurrent_mA = min(reqCurrent, maxCurrentLimit_mA);
        Setpoint = targetCurrent_mA;
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/setpid", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp") && request->hasParam("ki") && request->hasParam("kd")) {
        Kp = request->getParam("kp")->value().toDouble();
        Ki = request->getParam("ki")->value().toDouble();
        Kd = request->getParam("kd")->value().toDouble();
        myPID.SetTunings(Kp, Ki, Kd);
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
  });

  // --- NEW: Route to set advanced parameters ---
  server.on("/setadvanced", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("max")) {
        maxCurrentLimit_mA = request->getParam("max")->value().toDouble();
        ina219.setMaxCurrentShunt(maxCurrentLimit_mA / 1000.0, SHUNT_RESISTOR_OHMS);
        if (targetCurrent_mA > maxCurrentLimit_mA) {
            targetCurrent_mA = maxCurrentLimit_mA;
            Setpoint = targetCurrent_mA;
        }
    }
    if (request->hasParam("interval")) {
        float intervalSeconds = request->getParam("interval")->value().toFloat();
        // Enforce a minimum of 0.1 seconds (100 ms)
        if (intervalSeconds < 0.1) {
            intervalSeconds = 0.1;
        }
        sseInterval = (unsigned long)(intervalSeconds * 1000);
    }
    request->send(200, "text/plain", "OK");
  });

  // --- Server-Sent Events Setup ---
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);
  
  // --- Start Server ---
  server.begin();

  // --- Initialize PID ---
  Setpoint = targetCurrent_mA;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  // --- Read Sensors ---
  busVoltage_V = ina219.getBusVoltage();
  current_mA = ina219.getCurrent_mA();

  // --- Safety Check: Voltage Limit Override ---
  if (busVoltage_V >= MAXIMUM_BUS_VOLTAGE_INA219 && targetCurrent_mA > current_mA) {
    dacWrite(DAC_PIN, DAC_SAFETY_VALUE);
  } else {
    // --- Normal PID Operation ---
    Input = current_mA;
    myPID.Compute();
    setBuckFeedback(Output);
  }

  // --- Send Data via SSE at the specified interval ---
  static unsigned long lastEventTime = 0;
  if (millis() - lastEventTime > sseInterval) {
    lastEventTime = millis();
    
    // Create a JSON string with all relevant data
    String jsonData = "{";
    jsonData += "\"voltage\":" + String(busVoltage_V, 2);
    jsonData += ", \"current\":" + String(current_mA, 2);
    jsonData += ", \"setpoint\":" + String(targetCurrent_mA, 2);
    jsonData += ", \"kp\":" + String(Kp);
    jsonData += ", \"ki\":" + String(Ki);
    jsonData += ", \"kd\":" + String(Kd);
    jsonData += ", \"max_limit\":" + String(maxCurrentLimit_mA);
    jsonData += ", \"sse_interval\":" + String(sseInterval / 1000.0, 1);
    jsonData += "}";
    
    events.send(jsonData.c_str(), "message", millis());
  }

  delay(10);
}
