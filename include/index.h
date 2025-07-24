#pragma once

// index.h

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP Current Source</title>
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
    button { background-color: #1877f2; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; transition: background-color 0.2s; }
    button:hover { background-color: #166fe5; }
    button:disabled { background-color: #cccccc; }
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
    <h2>ESP Constant Current Source Controller</h2>
    
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
            <button onclick="setTargetCurrent(this)">Set</button>
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
        <button onclick="setPIDTunings(this)" style="width: 100%; margin-top: 10px;">Set Tunings</button>
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
                <label for="updateInterval">Update Interval (s):</label>
                <input type="number" id="updateInterval" min="0.1" step="0.1" value="1.0">
            </div>
            <div class="setting-group">
                <label for="chartPoints">Chart History (points):</label>
                <input type="number" id="chartPoints" min="10" step="10">
            </div>
            <button onclick="setAdvancedSettings(this)" style="width: 100%; margin-top: 10px;">Set Advanced</button>
        </div>
    </details>
  </div>

<script>
let chart;
let chartDataPoints = 60;
let updateIntervalHandle;
let updateIntervalMs = 1000;

function fetchData() {
    fetch('/data')
      .then(response => response.ok ? response.json() : Promise.reject('Network response was not ok'))
      .then(data => updateUI(data))
      .catch(error => console.error('Error fetching data:', error));
}

function updateUI(data) {
    document.getElementById('voltage').innerText = data.voltage;
    document.getElementById('current').innerText = data.current;
    
    document.getElementById('voltageWarning').style.display = data.voltage >= 25 ? 'inline' : 'none';
    
    const activeId = document.activeElement.id;
    if (activeId !== 'targetCurrentInput' && activeId !== 'targetCurrentSlider') {
        document.getElementById('targetCurrentInput').value = data.setpoint;
        document.getElementById('targetCurrentSlider').value = data.setpoint;
    }
    
    if (activeId !== 'maxCurrent') document.getElementById('maxCurrent').value = data.max_limit;
    
    document.getElementById('targetCurrentSlider').max = data.max_limit;
    document.getElementById('targetCurrentInput').max = data.max_limit;

    if (activeId !== 'kp') document.getElementById('kp').value = data.kp;
    if (activeId !== 'ki') document.getElementById('ki').value = data.ki;
    if (activeId !== 'kd') document.getElementById('kd').value = data.kd;
    
    const time = new Date().toLocaleTimeString();
    addDataToChart(time, data.current, data.setpoint, data.voltage);
}

function showButtonFeedback(button, originalText, success) {
    if (success) {
        button.innerText = 'Updated!';
        button.style.backgroundColor = '#28a745';
    } else {
        button.innerText = 'Failed!';
        button.style.backgroundColor = '#dc3545'; // Red for error
    }
    button.disabled = true;

    setTimeout(() => {
        button.innerText = originalText;
        button.style.backgroundColor = '#1877f2';
        button.disabled = false;
    }, 1500);
}

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
      animation: { duration: 0 }
    }
  });
  
  fetchData();
  updateIntervalHandle = setInterval(fetchData, updateIntervalMs);
};

function updateInputFromSlider(value) { document.getElementById('targetCurrentInput').value = value; }
function updateSliderFromInput() { document.getElementById('targetCurrentSlider').value = document.getElementById('targetCurrentInput').value; }

function setTargetCurrent(button) {
  var value = document.getElementById('targetCurrentInput').value;
  fetch(`/set?current=${value}`)
    .then(response => showButtonFeedback(button, 'Set', response.ok))
    .catch(err => showButtonFeedback(button, 'Set', false));
}

function setPIDTunings(button) {
    var kp = document.getElementById('kp').value;
    var ki = document.getElementById('ki').value;
    var kd = document.getElementById('kd').value;
    fetch(`/setpid?kp=${kp}&ki=${ki}&kd=${kd}`)
     .then(response => showButtonFeedback(button, 'Set Tunings', response.ok))
     .catch(err => showButtonFeedback(button, 'Set Tunings', false));
}

function setAdvancedSettings(button) {
    var max = document.getElementById('maxCurrent').value;
    var interval = document.getElementById('updateInterval').value;
    chartDataPoints = document.getElementById('chartPoints').value;
    
    if (interval < 0.1) interval = 0.1;
    updateIntervalMs = interval * 1000;
    clearInterval(updateIntervalHandle);
    updateIntervalHandle = setInterval(fetchData, updateIntervalMs);
    
    fetch(`/setadvanced?max=${max}`)
     .then(response => showButtonFeedback(button, 'Set Advanced', response.ok))
     .catch(err => showButtonFeedback(button, 'Set Advanced', false));
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
</script>
</body>
</html>
)rawliteral";
