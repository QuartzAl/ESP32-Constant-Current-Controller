// ================================================================= //
//      ESP32 Constant Current Source Controller Firmware (v7)     //
// ================================================================= //
//
// This firmware uses an ESP32 to create a constant current source.
// It features a PID controller, a web interface for control and
// monitoring, and uses Server-Sent Events for live data updates.
//
// v7 Changes:
// - Added expandable "Advanced Settings" tab on the webpage.
// - Moved "Max Current Limit" setting into the new advanced tab.
// - Added controls to change SSE update interval and chart data points.
// - Added a client-side warning that appears when voltage exceeds 25V.
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

// --- Pin Definitions ---
const int DAC_PIN = 25; // DAC output pin (GPIO25)

// --- INA219 Sensor Configuration ---
const uint8_t INA219_ADDRESS = 0x40;
const float SHUNT_RESISTOR_OHMS = 0.1;
const float MAXIMUM_BUS_VOLTAGE_INA219 = 25.0; // Safety limit for INA219

INA219 ina219(INA219_ADDRESS);

// --- PID Controller ---
double Setpoint, Input, Output; // PID variables
double Kp = 20, Ki = 5, Kd = 1;   // PID tuning parameters
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

// --- Buck Converter Feedback Voltage ---
const float BUCK_FEEDBACK_VOLTAGE = 1.25;
const int DAC_SAFETY_VALUE = (int)((BUCK_FEEDBACK_VOLTAGE / 3.3) * 255.0); // Approx. 97

// --- HTML for the Web Page ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Current Source</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f2f5; color: #333; }
    .container { max-width: 800px; margin: auto; background: white; padding: 25px; border-radius: 10px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); }
    h2 { color: #1877f2; border-bottom: 2px solid #e7e7e7; padding-bottom: 10px; margin-bottom: 20px; }
    .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 30px; }
    .card { background: #f8f9fa; padding: 20px; border-radius: 8px; }
    .card h3 { margin-top: 0; color: #444; }
    .sensor-data p { background: #e9ecef; padding: 12px; border-radius: 5px; display: flex; justify-content: space-between; align-items: center; }
    .sensor-data span { font-weight: bold; color: #0056b3; }
    label { display: block; margin-bottom: 8px; font-weight: 500; }
    input[type="range"] { width: 100%; -webkit-appearance: none; appearance: none; height: 8px; background: #ddd; border-radius: 5px; outline: none; margin-top: 5px; }
    input[type="range"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; background: #1877f2; cursor: pointer; border-radius: 50%; }
    input[type="range"]::-moz-range-thumb { width: 20px; height: 20px; background: #1877f2; cursor: pointer; border-radius: 50%; }
    input[type="number"] { width: 80px; padding: 8px 10px; border-radius: 5px; border: 1px solid #ccc; }
    .control-group, .setting-group { display: flex; justify-content: space-between; align-items: center; gap: 15px; margin-bottom: 15px; }
    button { background-color: #1877f2; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; }
    button:hover { background-color: #166fe5; }
    .pid-inputs { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
    .chart-container { margin-top: 30px; }
    .download-btn { background-color: #28a745; margin-top: 15px; width: 100%; }
    .download-btn:hover { background-color: #218838; }
    details > summary { font-weight: bold; cursor: pointer; padding: 10px; border-radius: 5px; background-color: #e9ecef; }
    #voltageWarning { color: #ffc107; font-weight: bold; margin-left: 10px; display: none; }
  </style>
</head>
<body>
  <div class="container">
    <h2>ESP32 Constant Current Source Controller</h2>
    
    <div class="grid-container">
      <div class="card">
        <h3>Live Data</h3>
        <div class="sensor-data">
          <p>Output Voltage: <span id="voltage">--</span> V <span id="voltageWarning">&#9888; LIMIT</span></p>
          <p>Measured Current: <span id="current">--</span> mA</p>
        </div>
      </div>

      <div class="card">
        <h3>Controls</h3>
        <div class="control-group">
            <label for="targetCurrentInput">Target (mA):</label>
            <input type="number" id="targetCurrentInput" min="0" step="1" onchange="updateSliderFromInput()">
            <button onclick="setTargetCurrent()">Set</button>
        </div>
        <input type="range" id="targetCurrentSlider" min="0" step="1" oninput="updateInputFromSlider(this.value)">
      </div>
    </div>

    <div class="card" style="margin-top: 20px;">
        <h3>PID Tuning</h3>
        <div class="pid-inputs">
            <div><label for="kp">Kp</label><input type="number" id="kp" step="0.1"></div>
            <div><label for="ki">Ki</label><input type="number" id="ki" step="0.1"></div>
            <div><label for="kd">Kd</label><input type="number" id="kd" step="0.1"></div>
        </div>
        <button onclick="setPIDTunings()" style="width: 100%; margin-top: 10px;">Set Tunings</button>
    </div>

    <div class="chart-container">
      <canvas id="currentChart"></canvas>
      <button class="download-btn" onclick="downloadCSV()">Download Chart Data (CSV)</button>
    </div>

    <details style="margin-top: 20px;">
        <summary>Advanced Settings</summary>
        <div class="card" style="margin-top: 10px;">
            <div class="setting-group">
                <label for="maxCurrent">Max Current Limit (mA):</label>
                <input type="number" id="maxCurrent" step="10">
            </div>
            <div class="setting-group">
                <label for="sseInterval">Update Interval (s):</label>
                <input type="number" id="sseInterval" min="0.1" step="0.1">
            </div>
            <div class="setting-group">
                <label for="chartPoints">Chart History (points):</label>
                <input type="number" id="chartPoints" min="10" step="10">
            </div>
            <button onclick="setAdvancedSettings()" style="width: 100%; margin-top: 10px;">Set Advanced</button>
        </div>
    </details>
  </div>

<script>
let chart;
let chartDataPoints = 60; // Default value

window.onload = function() {
  document.getElementById('chartPoints').value = chartDataPoints;
  const ctx = document.getElementById('currentChart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [
        { label: 'Measured Current (mA)', yAxisID: 'yCurrent', data: [], borderColor: 'rgba(24, 119, 242, 1)', backgroundColor: 'rgba(24, 119, 242, 0.2)', borderWidth: 2, fill: true, tension: 0.4 }, 
        { label: 'Setpoint (mA)', yAxisID: 'yCurrent', data: [], borderColor: 'rgba(40, 167, 69, 1)', borderWidth: 2, borderDash: [5, 5], fill: false },
		{ label: 'Measured Voltage (V)', yAxisID: 'yVoltage', data: [], borderColor: 'rgba(219, 59, 55, 1)', backgroundColor: 'rgba(219, 59, 55, 0.2)', borderWidth: 2, fill: true, tension: 0.4 }
    ]},
    options: {
      scales: { 
          x: { title: { display: true, text: 'Time' } }, 
          yCurrent: { type: 'linear', position: 'left', title: { display: true, text: 'Current (mA)' }, beginAtZero: true },
          yVoltage: { type: 'linear', position: 'right', title: { display: true, text: 'Voltage (V)' }, beginAtZero: true, grid: { drawOnChartArea: false } }
      },
      animation: { duration: 250 }
    }
  });
};

function updateInputFromSlider(value) { document.getElementById('targetCurrentInput').value = value; }
function updateSliderFromInput() { document.getElementById('targetCurrentSlider').value = document.getElementById('targetCurrentInput').value; }

function setTargetCurrent() {
  var value = document.getElementById('targetCurrentInput').value;
  fetch(`/set?current=${value}`).catch(err => console.error(err));
}

function setPIDTunings() {
    var kp = document.getElementById('kp').value;
    var ki = document.getElementById('ki').value;
    var kd = document.getElementById('kd').value;
    fetch(`/setpid?kp=${kp}&ki=${ki}&kd=${kd}`).catch(err => console.error(err));
}

function setAdvancedSettings() {
    var max = document.getElementById('maxCurrent').value;
    var interval = document.getElementById('sseInterval').value;
    chartDataPoints = document.getElementById('chartPoints').value;
    
    document.getElementById('targetCurrentSlider').max = max;
    document.getElementById('targetCurrentInput').max = max;

    fetch(`/setadvanced?max=${max}&interval=${interval}`).catch(err => console.error(err));
}

function addDataToChart(label, current, setpoint, voltage) {
    chart.data.labels.push(label);
    chart.data.datasets[0].data.push(current);
    chart.data.datasets[1].data.push(setpoint);
	chart.data.datasets[2].data.push(voltage);
    while(chart.data.labels.length > chartDataPoints) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(dataset => dataset.data.shift());
    }
    chart.update('none');
}

function downloadCSV() {
    let csvContent = "data:text/csv;charset=utf-8,Time,Measured Current (mA),Setpoint (mA),Measured Voltage (V)\n";
    const labels = chart.data.labels;
    const currentData = chart.data.datasets[0].data;
    const setpointData = chart.data.datasets[1].data;
    const voltageData = chart.data.datasets[2].data;
    for (let i = 0; i < labels.length; i++) {
        csvContent += `${labels[i]},${currentData[i]},${setpointData[i]},${voltageData[i]}\n`;
    }
    var encodedUri = encodeURI(csvContent);
    var link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", "power_supply_data.csv");
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

if (!!window.EventSource) {
  var source = new EventSource('/events');
  source.onopen = e => console.log("Events Connected");
  source.onerror = e => { if (e.target.readyState != EventSource.OPEN) console.log("Events Disconnected"); };
  source.onmessage = e => {
    const data = JSON.parse(e.data);
    document.getElementById('voltage').innerText = data.voltage;
    document.getElementById('current').innerText = data.current;
    
    // Client-side voltage warning
    if (data.voltage >= 25) {
        document.getElementById('voltageWarning').style.display = 'inline';
    } else {
        document.getElementById('voltageWarning').style.display = 'none';
    }
    
    document.getElementById('targetCurrentInput').value = data.setpoint;
    document.getElementById('targetCurrentSlider').value = data.setpoint;
    
    document.getElementById('maxCurrent').value = data.max_limit;
    document.getElementById('targetCurrentSlider').max = data.max_limit;
    document.getElementById('targetCurrentInput').max = data.max_limit;
    
    document.getElementById('sseInterval').value = data.sse_interval;

    document.getElementById('kp').value = data.kp;
    document.getElementById('ki').value = data.ki;
    document.getElementById('kd').value = data.kd;
    
    const time = new Date().toLocaleTimeString();
    addDataToChart(time, data.current, data.setpoint, data.voltage);
  };
}
</script>
</body>
</html>
)rawliteral";

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

  // --- WiFiManager Setup ---
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
