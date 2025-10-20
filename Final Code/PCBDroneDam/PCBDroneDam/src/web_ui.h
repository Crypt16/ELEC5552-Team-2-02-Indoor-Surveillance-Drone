#pragma once
// web_ui.h - Navigation System Web Interface

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Navigation System</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  padding: 15px;
  background: #f5f5f5;
  color: #333;
}
.header {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  padding: 15px;
  border-radius: 10px;
  margin-bottom: 15px;
}
h1 { font-size: 1.5rem; margin-bottom: 5px; }
.subtitle { opacity: 0.9; font-size: 0.85rem; }
.card {
  background: white;
  border-radius: 10px;
  padding: 15px;
  margin-bottom: 15px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}
.card h2 {
  margin: 0 0 12px 0;
  font-size: 1.1rem;
  color: #667eea;
  border-bottom: 2px solid #667eea;
  padding-bottom: 5px;
}
.sensor-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
}
.sensor-item {
  text-align: center;
  padding: 12px;
  background: #f8f9fa;
  border-radius: 8px;
  border: 2px solid #ddd;
}
.sensor-label {
  font-size: 0.7rem;
  color: #666;
  font-weight: 600;
  text-transform: uppercase;
  margin-bottom: 5px;
}
.sensor-value {
  font-size: 1.3rem;
  font-weight: bold;
  color: #667eea;
}
.sensor-item.obstacle {
  background: #ff6b6b;
  border-color: #c92a2a;
  animation: pulse 1s ease-in-out infinite;
}
.sensor-item.obstacle .sensor-value,
.sensor-item.obstacle .sensor-label {
  color: white;
}
@keyframes pulse {
  0%, 100% { transform: scale(1); }
  50% { transform: scale(1.05); }
}
.position-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(90px, 1fr));
  gap: 10px;
}
.position-item {
  text-align: center;
  padding: 10px;
  background: #f8f9fa;
  border-radius: 8px;
}
.position-label {
  font-size: 0.7rem;
  color: #666;
  margin-bottom: 4px;
  font-weight: 600;
}
.position-value {
  font-size: 1.1rem;
  font-weight: bold;
  color: #764ba2;
}
.status-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  flex-wrap: wrap;
  gap: 10px;
  margin-bottom: 10px;
}
.mode-badge {
  display: inline-block;
  padding: 6px 12px;
  border-radius: 15px;
  font-weight: 600;
  font-size: 0.85rem;
  text-transform: uppercase;
}
.mode-navigate { background: #28a745; color: white; }
.mode-hover { background: #ffc107; color: #000; }
.waypoint-form {
  display: grid;
  grid-template-columns: 1fr 1fr auto;
  gap: 8px;
}
.waypoint-form input {
  padding: 8px;
  border: 2px solid #ddd;
  border-radius: 6px;
  font-size: 0.9rem;
}
.waypoint-form button {
  padding: 8px 15px;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 6px;
  font-weight: 600;
  cursor: pointer;
}
.waypoint-form button:active {
  background: #5568d3;
}
.calibrate-btn {
  width: 100%;
  padding: 12px;
  background: #ff6b6b;
  color: white;
  border: none;
  border-radius: 6px;
  font-weight: 600;
  font-size: 0.95rem;
  cursor: pointer;
  margin-top: 10px;
}
.calibrate-btn:active {
  background: #c92a2a;
}
.log-box {
  background: #1e1e1e;
  color: #00ff00;
  padding: 12px;
  border-radius: 8px;
  font-family: 'Courier New', monospace;
  font-size: 0.75rem;
  max-height: 200px;
  overflow-y: auto;
}
.log-line { margin: 2px 0; }
@media (max-width: 600px) {
  .sensor-grid { grid-template-columns: repeat(2, 1fr); }
  .position-grid { grid-template-columns: repeat(2, 1fr); }
  .waypoint-form { grid-template-columns: 1fr; }
}
</style>
</head>
<body>

<div class="header">
  <h1>🚁 Navigation System</h1>
  <div class="subtitle">Real-time Sensor Monitoring</div>
</div>

<div class="card">
  <h2>System Status</h2>
  <div class="status-row">
    <div><strong>Mode:</strong> <span id="mode" class="mode-badge mode-navigate">NAVIGATE</span></div>
    <div><strong>Active:</strong> <span id="active">No</span></div>
    <div><strong>Queue:</strong> <span id="queue">0</span></div>
  </div>
</div>

<div class="card">
  <h2>Obstacle Detection</h2>
  <div class="sensor-grid">
    <div class="sensor-item" id="sensor-left">
      <div class="sensor-label">Left</div>
      <div class="sensor-value">—</div>
    </div>
    <div class="sensor-item" id="sensor-forward">
      <div class="sensor-label">Forward</div>
      <div class="sensor-value">—</div>
    </div>
    <div class="sensor-item" id="sensor-right">
      <div class="sensor-label">Right</div>
      <div class="sensor-value">—</div>
    </div>
  </div>
</div>

<div class="card">
  <h2>Height Sensor (Bottom)</h2>
  <div class="sensor-grid" style="grid-template-columns: 1fr;">
    <div class="sensor-item" id="sensor-bottom-left">
      <div class="sensor-label">Bottom Left</div>
      <div class="sensor-value">—</div>
    </div>
  </div>
</div>

<div class="card">
  <h2>Position & Velocity</h2>
  <div class="position-grid">
    <div class="position-item">
      <div class="position-label">X</div>
      <div class="position-value" id="pos-x">0.000 m</div>
    </div>
    <div class="position-item">
      <div class="position-label">Y</div>
      <div class="position-value" id="pos-y">0.000 m</div>
    </div>
    <div class="position-item">
      <div class="position-label">Height</div>
      <div class="position-value" id="height">0.000 m</div>
    </div>
    <div class="position-item">
      <div class="position-label">Vx</div>
      <div class="position-value" id="vel-x">0 mm/s</div>
    </div>
    <div class="position-item">
      <div class="position-label">Vy</div>
      <div class="position-value" id="vel-y">0 mm/s</div>
    </div>
  </div>
</div>

<div class="card">
  <h2>IMU (MPU6500)</h2>
  <div class="position-grid">
    <div class="position-item">
      <div class="position-label">Roll</div>
      <div class="position-value" id="imu-roll">—</div>
    </div>
    <div class="position-item">
      <div class="position-label">Pitch</div>
      <div class="position-value" id="imu-pitch">—</div>
    </div>
    <div class="position-item">
      <div class="position-label">Yaw</div>
      <div class="position-value" id="imu-yaw">—</div>
    </div>
    <div class="position-item">
      <div class="position-label">Accel</div>
      <div class="position-value" id="imu-accel">—</div>
    </div>
  </div>
  <div style="margin-top: 10px; font-size: 0.8rem; color: #666; text-align: center;">
    <span id="imu-xyz">X: — | Y: — | Z: —</span>
  </div>
</div>

<div class="card">
  <h2>Calibration</h2>
  <button class="calibrate-btn" onclick="calibrateSensors()">🔄 Clear All Sensor Buffers</button>
  <div style="margin-top: 8px; font-size: 0.75rem; color: #666; text-align: center;">
    Clears all buffered sensor data to reset filtering
  </div>
</div>

<div class="card">
  <h2>Waypoint Control</h2>
  <div class="waypoint-form">
    <input type="number" id="wp-dx" placeholder="ΔX (m)" step="0.1" value="0">
    <input type="number" id="wp-dy" placeholder="ΔY (m)" step="0.1" value="0">
    <button onclick="addWaypoint()">Add</button>
  </div>
</div>

<div class="card">
  <h2>System Log</h2>
  <div class="log-box" id="log"></div>
</div>

<script>
const $ = id => document.getElementById(id);
let socket = null;
const logLines = [];
const MAX_LOG = 50;

function log(msg, color = '#00ff00') {
  const time = new Date().toLocaleTimeString();
  logLines.push(`<div class="log-line" style="color:${color}">[${time}] ${msg}</div>`);
  if (logLines.length > MAX_LOG) logLines.shift();
  $('log').innerHTML = logLines.join('');
  $('log').scrollTop = $('log').scrollHeight;
}

function updateSensor(id, value, threshold) {
  const elem = $(`sensor-${id}`);
  const valueElem = elem.querySelector('.sensor-value');
  
  if (value === 9999 || value === null) {
    valueElem.textContent = '—';
    elem.classList.remove('obstacle');
  } else {
    valueElem.textContent = value + 'mm';
    if (threshold > 0 && value <= threshold) {
      elem.classList.add('obstacle');
    } else {
      elem.classList.remove('obstacle');
    }
  }
}

function calibrateSensors() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send('CALIBRATE');
    log('Calibrating - clearing all buffers', '#ff9900');
  } else {
    log('Not connected', '#ff0000');
  }
}

function connectWebSocket() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  socket = new WebSocket(`${proto}://${location.host}/ws`);
  
  socket.onopen = () => log('Connected', '#00ff00');
  socket.onclose = () => {
    log('Disconnected, reconnecting...', '#ff9900');
    setTimeout(connectWebSocket, 2000);
  };
  socket.onerror = () => log('Connection error', '#ff0000');
  
  socket.onmessage = (e) => {
    const msg = e.data.trim();
    
    if (msg.startsWith('RANGES')) {
      const parts = msg.split(' ');
      for (let i = 1; i < parts.length; i++) {
        const [k, v] = parts[i].split(':');
        const val = parseInt(v);
        if (k === 'L') updateSensor('left', val, 500);
        else if (k === 'R') updateSensor('right', val, 500);
        else if (k === 'F') updateSensor('forward', val, 500);
        else if (k === 'BL') updateSensor('bottom-left', val, 0);
      }
    }
    else if (msg.startsWith('POS')) {
      const m = msg.match(/X:([-\d.]+)\s+Y:([-\d.]+)\s+H:([-\d.]+)\s+VX:([-\d]+)\s+VY:([-\d]+)/);
      if (m) {
        $('pos-x').textContent = parseFloat(m[1]).toFixed(3) + ' m';
        $('pos-y').textContent = parseFloat(m[2]).toFixed(3) + ' m';
        $('height').textContent = parseFloat(m[3]).toFixed(3) + ' m';
        $('vel-x').textContent = m[4] + ' mm/s';
        $('vel-y').textContent = m[5] + ' mm/s';
      }
    }
    else if (msg.startsWith('MODE')) {
      const mode = msg.split(' ')[1];
      const badge = $('mode');
      badge.textContent = mode;
      badge.className = 'mode-badge mode-' + mode.toLowerCase();
    }
    else if (msg.startsWith('STATUS')) {
      const m = msg.match(/Active:(\w+)\s+Queue:(\d+)/);
      if (m) {
        $('active').textContent = m[1] === 'Y' ? 'Yes' : 'No';
        $('queue').textContent = m[2];
      }
    }
    else if (msg.startsWith('IMU')) {
      const m = msg.match(/R:([-\d.]+)\s+P:([-\d.]+)\s+Y:([-\d.]+)\s+AX:([-\d.]+)\s+AY:([-\d.]+)\s+AZ:([-\d.]+)/);
      if (m) {
        $('imu-roll').textContent = parseFloat(m[1]).toFixed(1) + '°';
        $('imu-pitch').textContent = parseFloat(m[2]).toFixed(1) + '°';
        $('imu-yaw').textContent = parseFloat(m[3]).toFixed(1) + '°';
        
        const ax = parseFloat(m[4]);
        const ay = parseFloat(m[5]);
        const az = parseFloat(m[6]);
        const accel_mag = Math.sqrt(ax*ax + ay*ay + az*az);
        $('imu-accel').textContent = accel_mag.toFixed(2) + 'g';
        $('imu-xyz').textContent = `X: ${ax.toFixed(2)}g | Y: ${ay.toFixed(2)}g | Z: ${az.toFixed(2)}g`;
      }
    }
    else {
      log(msg);
    }
  };
}

function addWaypoint() {
  const dx = parseFloat($('wp-dx').value) || 0;
  const dy = parseFloat($('wp-dy').value) || 0;
  
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(`WAYPOINT:move ${dx} ${dy}`);
    log(`Waypoint: ΔX=${dx}m, ΔY=${dy}m`, '#00aaff');
    $('wp-dx').value = '0';
    $('wp-dy').value = '0';
  } else {
    log('Not connected', '#ff0000');
  }
}

connectWebSocket();
log('Interface loaded', '#00aaff');
</script>

</body>
</html>
)HTML";