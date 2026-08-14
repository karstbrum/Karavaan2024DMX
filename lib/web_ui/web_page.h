#ifndef WEB_PAGE_H
#define WEB_PAGE_H

// Single-page LED troubleshooting UI: plots pixel_pos on a canvas, colors
// the dots from live RGBW state, and lets a mode + its parameters be sent
// to the board to trigger an effect without DMX/ESP-NOW input.
static const char WEB_UI_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LED troubleshooter</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:12px; }
  h1 { font-size:16px; margin:0 0 10px; }
  #layout { display:flex; flex-wrap:wrap; gap:16px; }
  canvas { background:#000; border:1px solid #444; border-radius:4px; }
  .panel { min-width:260px; }
  .row { display:flex; align-items:center; gap:8px; margin:6px 0; }
  .row label { width:70px; font-size:13px; }
  .row input[type=range] { flex:1; }
  .row output { width:34px; text-align:right; font-size:12px; }
  select, button { font-size:14px; padding:4px; }
  button { cursor:pointer; margin-top:10px; }
</style>
</head>
<body>
<h1>LED troubleshooter</h1>
<div id="layout">
  <canvas id="c" width="480" height="480"></canvas>
  <div class="panel">
    <div class="row">
      <label for="mode">Mode</label>
      <select id="mode">
        <option value="0">0 - Strobe</option>
        <option value="1">1 - Alternate clusters</option>
        <option value="2">2 - Flashing pixels</option>
        <option value="3">3 - Moving lines</option>
        <option value="4">4 - Rotation</option>
        <option value="5">5 - Rainbow</option>
      </select>
    </div>
    <div class="row"><label for="extra1">Extra1</label><input type="range" id="extra1" min="0" max="255" value="0"><output></output></div>
    <div class="row"><label for="bpm">BPM</label><input type="range" id="bpm" min="1" max="255" value="100"><output></output></div>
    <div class="row"><label for="dim">Dim</label><input type="range" id="dim" min="0" max="255" value="255"><output></output></div>
    <div class="row"><label for="dimmer">Fadetime</label><input type="range" id="dimmer" min="0" max="255" value="0"><output></output></div>
    <div class="row"><label for="color">Color</label><input type="range" id="color" min="0" max="255" value="255"><output></output></div>
    <button id="send">Send</button>
  </div>
</div>
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let xs = [], ys = [];

function project(x, y) {
  const pad = 20;
  const size = Math.min(canvas.width, canvas.height) - pad * 2;
  return {
    px: pad + (x - project.minX) / (project.maxX - project.minX || 1) * size,
    py: pad + (1 - (y - project.minY) / (project.maxY - project.minY || 1)) * size
  };
}

function draw(colors) {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  for (let i = 0; i < xs.length; i++) {
    const p = project(xs[i], ys[i]);
    if (colors) {
      const [w, r, g, b] = colors[i];
      ctx.fillStyle = `rgb(${Math.min(255, r + w)},${Math.min(255, g + w)},${Math.min(255, b + w)})`;
    } else {
      ctx.fillStyle = '#555';
    }
    ctx.beginPath();
    ctx.arc(p.px, p.py, 3, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// The ESP32 WebServer sends "Connection: close" on every response (it's
// hardcoded in the library, not something we control from here), so every
// poll is a full TCP connect/request/close cycle, not a reused persistent
// connection. Polling much faster than this risks exhausting the ESP32's
// small socket/heap budget over a long-running session and hanging the
// board. Raise this only with extended soak testing to confirm stability.
const POLL_INTERVAL = 30;

// self-pacing poll: only issues the next /colors request once the previous
// one has finished (or failed), instead of firing on a fixed setInterval.
// The ESP32's HTTP server is synchronous and single-threaded, so firing
// requests on a fixed timer regardless of completion lets a backlog build
// up faster than it can be drained - eventually starving out other
// requests like /set until the page is reloaded.
async function pollColors() {
  try {
    const r = await fetch('/colors');
    draw(await r.json());
  } catch (e) {
    // transient failure (e.g. server busy) - just retry next tick
  }
  setTimeout(pollColors, POLL_INTERVAL);
}

fetch('/positions').then(r => r.json()).then(data => {
  xs = data.x;
  ys = data.y;
  project.minX = Math.min(...xs);
  project.maxX = Math.max(...xs);
  project.minY = Math.min(...ys);
  project.maxY = Math.max(...ys);
  draw(null);
  pollColors();
});

for (const id of ['extra1', 'bpm','dim','dimmer','color']) {
  const input = document.getElementById(id);
  const out = input.nextElementSibling;
  out.textContent = input.value;
  input.addEventListener('input', () => out.textContent = input.value);
}

document.getElementById('send').addEventListener('click', () => {
  const button = document.getElementById('send');
  const body = new URLSearchParams();
  for (const id of ['mode', 'extra1', 'bpm','dim','dimmer','color']) {
    body.set(id, document.getElementById(id).value);
  }
  button.textContent = 'Sending...';
  fetch('/set', { method: 'POST', body })
    .then(() => { button.textContent = 'Send'; })
    .catch(() => { button.textContent = 'Failed, retry'; });
});
</script>
</body>
</html>
)rawliteral";

#endif
