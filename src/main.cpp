// ================================================================= //
//      ESP32 Constant Current Source Controller Firmware          //
// =================================----------------================ //
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
//
// Installation:
// 1. Open the Arduino IDE.
// 2. Go to Sketch > Include Library > Manage Libraries.
// 3. Search for and install the following libraries:
//    - "ESPAsyncWebServer"
//    - "AsyncTCP" (a dependency for ESPAsyncWebServer)
//    - "WiFiManager" by tzapu
//    - "INA219" by Rob Tillaart
//    - "PID" by Brett Beauregard
//
// Hardware Connections:
// - ESP32 Pin 25 (DAC1) -> Buck Converter Feedback Pin
// - ESP32 Pin 21 (SDA)  -> INA219 SDA Pin
// - ESP32 Pin 22 (SCL)  -> INA219 SCL Pin
// - INA219 VCC -> 3.3V
// - INA219 GND -> GND

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <INA219.h>
#include <PID_v1.h>

// --- Pin Definitions ---
const int DAC_PIN = 25; // DAC output pin (GPIO25)

// --- INA219 Sensor ---
INA219 ina219;

// --- PID Controller ---
double Setpoint, Input, Output; // PID variables
double Kp = 20, Ki = 5, Kd = 1;   // PID tuning parameters (start with these and tune)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Web Server and SSE ---
AsyncWebServer server(80);
AsyncEventSource events("/events");

// --- Global Variables ---
float busVoltage_V = 0;
float current_mA = 0;
double targetCurrent_mA = 100.0; // Default target current in mA

// --- Buck Converter Feedback Voltage ---
// The buck converter's feedback reference voltage.
// This is crucial for scaling the DAC output correctly.
// Common values are 1.25V, 0.8V, 0.6V. Check your datasheet.
const float BUCK_FEEDBACK_VOLTAGE = 1.25;

// --- HTML for the Web Page ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Current Source</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; color: #333; }
    .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    h2 { color: #007BFF; }
    .sensor-data { margin-bottom: 20px; }
    .sensor-data p { background: #eee; padding: 10px; border-radius: 5px; }
    .sensor-data span { font-weight: bold; }
    .control { margin-top: 20px; }
    label { display: block; margin-bottom: 5px; }
    input[type="range"] { width: 100%; }
    #targetCurrentValue { font-weight: bold; color: #28a745; }
    button { background-color: #007BFF; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; }
    button:hover { background-color: #0056b3; }
  </style>
</head>
<body>
  <div class="container">
    <h2>ESP32 Constant Current Source Controller</h2>
    
    <div class="sensor-data">
      <p>Output Voltage: <span id="voltage">--</span> V</p>
      <p>Measured Current: <span id="current">--</span> mA</p>
    </div>

    <div class="control">
      <label for="targetCurrent">Target Current: <span id="targetCurrentValue">--</span> mA</label>
      <input type="range" id="targetCurrent" min="0" max="500" step="1" oninput="updateSliderValue(this.value)">
      <br><br>
      <button onclick="setTargetCurrent()">Set Current</button>
    </div>
  </div>

<script>
function updateSliderValue(value) {
  document.getElementById('targetCurrentValue').innerText = value;
}

function setTargetCurrent() {
  var value = document.getElementById('targetCurrent').value;
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/set?current=" + value, true);
  xhr.send();
}

if (!!window.EventSource) {
  var source = new EventSource('/events');
  
  source.onopen = function(e) {
    console.log("Events Connected");
  };

  source.onerror = function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  };
  
  source.onmessage = function(e) {
    console.log("message", e.data);
    var data = JSON.parse(e.data);
    document.getElementById('voltage').innerText = data.voltage;
    document.getElementById('current').innerText = data.current;
    if (document.getElementById('targetCurrentValue').innerText !== data.setpoint) {
        document.getElementById('targetCurrent').value = data.setpoint;
        document.getElementById('targetCurrentValue').innerText = data.setpoint;
    }
  };
}
</script>
</body>
</html>
)rawliteral";

// --- Function to scale PID output to DAC value ---
// The ESP32 DAC is 8-bit (0-255) and outputs 0-3.3V.
// We need to map the PID output to a DAC value that will
// adjust the buck converter's output.
// A lower feedback voltage generally means a higher output voltage/current.
// A higher feedback voltage generally means a lower output voltage/current.
// The PID output range is set from 0 to 255.
// We assume a lower DAC value (lower feedback voltage) increases current.
void setBuckFeedback(double pidOutput) {
    // The PID output is 0-255. We need to map this to a DAC value.
    // The relationship is often inverse: higher feedback voltage -> lower output current.
    // We clamp the output to a safe range to prevent extreme values.
    int dacValue = constrain((int)pidOutput, 0, 255);
    
    // The standard DAC output is Vout = (dacValue/255) * 3.3V.
    // We write the inverted value if needed, but here we assume DIRECT PID action
    // where higher output means higher current. If your setup is inverse,
    // you might need to use `255 - dacValue` or change PID to REVERSE.
    dacWrite(DAC_PIN, dacValue);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- Initialize INA219 ---
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // Configure INA219. The arguments are:
  // 1. Voltage range (16V or 32V)
  // 2. Max expected current in Amps
  // These values are used to calculate the best resolution.
  ina219.configure(INA219_RANGE_16V, 0.5); // 16V range, 500mA max current

  // --- Initialize DAC ---
  dacWrite(DAC_PIN, 0); // Start with DAC off

  // --- WiFiManager Setup ---
  WiFiManager wm;
  // wm.resetSettings(); // Uncomment to reset saved credentials
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
        String value = request->getParam("current")->value();
        targetCurrent_mA = value.toDouble();
        Setpoint = targetCurrent_mA; // Update PID setpoint
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
  });

  // --- Server-Sent Events Setup ---
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);
  
  // --- Start Server ---
  server.begin();

  // --- Initialize PID ---
  Setpoint = targetCurrent_mA;
  myPID.SetMode(AUTOMATIC);
  // The PID output range is 0-255, which matches the DAC resolution.
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  // --- Read Sensors ---
  busVoltage_V = ina219.getBusVoltage();
  current_mA = ina219.getCurrent();

  // --- Update PID Controller ---
  Input = current_mA;   // The measured current is the PID input
  myPID.Compute();      // Calculate the new output
  
  // --- Control the Buck Converter ---
  setBuckFeedback(Output);

  // --- Send Data via SSE every 1 second ---
  static unsigned long lastEventTime = 0;
  if (millis() - lastEventTime > 1000) {
    lastEventTime = millis();
    
    // Create a JSON string with the data
    String jsonData = "{";
    jsonData += "\"voltage\":" + String(busVoltage_V);
    jsonData += ", \"current\":" + String(current_mA);
    jsonData += ", \"setpoint\":" + String(targetCurrent_mA);
    jsonData += "}";
    
    events.send(jsonData.c_str(), "message", millis());
  }

  // A small delay to keep things stable
  delay(10);
}