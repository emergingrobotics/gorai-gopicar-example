// The page is a thin front-end: it POSTs control events and renders telemetry
// pushed over the WebSocket. It never talks to NATS directly (C-001).
const post = (t, v) => fetch("/control", {
  method: "POST", headers: { "Content-Type": "application/json" },
  body: JSON.stringify({ t, v }),
}).catch(() => {});

// Watchdog keep-alive: while a drive/steer input is engaged, re-send on interval
// so the component watchdog (C-003) is satisfied only while an operator is here.
let held = {};
setInterval(() => {
  for (const t of Object.keys(held)) post(t, held[t]);
}, 200);

const bind = (id, t, spring) => {
  const el = document.getElementById(id);
  const send = () => { held[t] = +el.value; post(t, +el.value); };
  el.addEventListener("input", send);
  if (spring) el.addEventListener("pointerup", () => { el.value = 0; delete held[t]; post(t, 0); });
};
bind("throttle", "drive", true);
bind("steer", "steer", true);
bind("campan", "campan", false);
bind("camtilt", "camtilt", false);
document.getElementById("stop").addEventListener("click", () => { held = {}; post("estop", 1); });

// Keyboard: same events as the sliders (I-003).
const keymap = {
  w: ["drive", 60], s: ["drive", -60], ArrowUp: ["drive", 60], ArrowDown: ["drive", -60],
  a: ["steer", -30], d: ["steer", 30], ArrowLeft: ["steer", -30], ArrowRight: ["steer", 30],
  i: ["camtilt", 20], k: ["camtilt", -20], j: ["campan", -20], l: ["campan", 20],
};
addEventListener("keydown", (e) => {
  if (e.key === " ") { held = {}; post("estop", 1); return; }
  if (e.key === "c") { post("centre", 0); post("campan", 0); post("camtilt", 0); return; }
  const m = keymap[e.key];
  if (m && !e.repeat) { held[m[0]] = m[1]; post(m[0], m[1]); }
});
addEventListener("keyup", (e) => {
  const m = keymap[e.key];
  if (!m) return;
  if (m[0] === "drive" || m[0] === "steer") { delete held[m[0]]; post(m[0], 0); }
});
addEventListener("blur", () => { held = {}; post("drive", 0); post("estop", 1); });
document.addEventListener("visibilitychange", () => { if (document.hidden) { held = {}; post("drive", 0); } });

// Telemetry over WebSocket.
const ws = new WebSocket(`ws://${location.host}/ws`);
ws.onmessage = (ev) => {
  const d = JSON.parse(ev.data);
  const set = (id, v) => { const e = document.getElementById(id); if (v !== undefined) e.textContent = v; };
  if (d.cap === "battery") set("battery", d.volts?.toFixed?.(2));
  if (d.cap === "distance") set("distance", d.cm);
  if (d.cap === "grayscale") set("grayscale", (d.adc || []).join(","));
  if (d.cap === "line") set("line", (d.line || []).join(","));
  if (d.cap === "cliff") { const e = document.getElementById("cliff"); e.textContent = d.cliff; e.className = d.cliff ? "alarm" : ""; }
  if (d.cap === "sysinfo") set("sysinfo", d.fw);
};
