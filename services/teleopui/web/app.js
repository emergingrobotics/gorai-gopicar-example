// The page is a thin front-end: it POSTs control events and renders telemetry
// pushed over the WebSocket. It never talks to NATS directly (C-001). The teleop-ui
// service translates each event to a NATS request on gorai.picarx.<cap>.command.
const el = (id) => document.getElementById(id);
const clamp = (s, v) => Math.max(+s.min, Math.min(+s.max, v));

// Show the reason a command was refused (e.g. estop_latched, cliff_blocked) so the
// controls never fail silently.
const setStatus = (msg, bad) => {
  const e = el("status");
  if (e) { e.textContent = msg || ""; e.className = bad ? "alarm" : ""; }
};

// post sends a control event and reports the tool's typed reply: on {ok:false}
// it surfaces the error; on success it clears any stale error.
const post = (t, v) => fetch("/control", {
  method: "POST", headers: { "Content-Type": "application/json" },
  body: JSON.stringify({ t, v }),
}).then((r) => r.json()).then((res) => {
  if (res && res.ok === false) setStatus(`${t} refused: ${res.error || "error"}`, true);
  else setStatus("");
  return res;
}).catch(() => {});

// Watchdog keep-alive: while a drive/steer input is engaged, re-send on interval
// so the component watchdog (C-003) is satisfied only while an operator is here.
let held = {};
setInterval(() => { for (const t of Object.keys(held)) post(t, held[t]); }, 200);

// Control channels: which slider they drive, whether they spring back to centre
// on release (steer only) or hold position (throttle + camera), and the keyboard
// step per press. drive/steer feed the drive watchdog, so a non-zero value must
// be re-sent by the keep-alive; camera does not.
const controls = {
  drive:   { slider: "throttle", spring: false, step: 10, keepAlive: true },
  steer:   { slider: "steer",    spring: true,  keyVal: 30, keepAlive: true },
  campan:  { slider: "campan",   spring: false, step: 8,  keepAlive: false },
  camtilt: { slider: "camtilt",  spring: false, step: 8,  keepAlive: false },
};

// apply is the single path that moves a control: it updates the slider so it
// visually tracks keyboard input (R-135/R-136), posts the command, and keeps the
// watchdog keep-alive alive only while the value is non-zero.
function apply(ch, value) {
  const c = controls[ch];
  const s = el(c.slider);
  const v = clamp(s, value);
  s.value = v;
  post(ch, v);
  if (c.keepAlive && v !== 0) held[ch] = v; else delete held[ch];
}

// step nudges a channel relative to its current slider position (for held
// controls: throttle, camera pan/tilt).
const step = (ch, dir) => apply(ch, +el(controls[ch].slider).value + dir * controls[ch].step);

// Slider (pointer/touch) input — identical channels and payloads to the keys (I-003).
for (const [ch, c] of Object.entries(controls)) {
  const s = el(c.slider);
  s.addEventListener("input", () => apply(ch, +s.value));
  if (c.spring) s.addEventListener("pointerup", () => apply(ch, 0));
}

// E-stop: engaging latches the motors off and stops them; it stays latched until
// explicitly cleared (R-153). The wire uses estop {clear:bool}: v=0 -> clear:false
// (engage + stop), v=1 -> clear:true (release). The button toggles engage/clear;
// the spacebar only ever engages (a safety stop must not be released by accident).
let estopped = false;
function setEstop(on) {
  estopped = on;
  held = {};
  if (on) { el("throttle").value = 0; el("steer").value = 0; }
  post("estop", on ? 0 : 1);
  const b = el("stop");
  // The button both fires the e-stop and shows whether it is engaged.
  b.textContent = on ? "⚠ E-STOP ENGAGED — click to clear" : "EMERGENCY STOP  (or press X)";
  b.classList.toggle("engaged", on);
}
el("stop").addEventListener("click", () => setEstop(!estopped));

// Soft stop: cut the throttle cruise and stop the motors, WITHOUT latching
// (unlike e-stop) — you can drive again immediately. Leaves steering as-is.
function softStop() {
  held = {};
  apply("drive", 0);
}

// Quit: stop the whole robot process from the GUI (graceful shutdown -> motors
// stopped, camera released). Confirmed so it can't fire by accident.
el("quit").addEventListener("click", () => {
  if (!confirm("Shut down the robot?")) return;
  held = {};
  fetch("/quit", { method: "POST" })
    .then(() => setStatus("robot shutting down…", false))
    .catch(() => {});
});

// Keyboard controls (active alongside the sliders, which track them visually):
//   Arrow Up/Down            throttle up/down        (steps and HOLDS the speed)
//   Arrow Left/Right         steer left/right        (springs to centre on release)
//   Shift + Arrow Up/Down    camera tilt up/down      (steps and holds)
//   Shift + Arrow Left/Right camera pan left/right    (steps and holds)
//   C                        centre everything (camera pan + tilt AND steering to 0)
//   Space                    stop the motors (no latch — drive again right away)
//   X                        e-stop (latches; clear with the on-screen button)
//   W/A/S/D                  mirror the arrows for throttle/steer (R-136)
const VKEYS = { ArrowUp: 1, ArrowDown: -1, w: 1, s: -1 };          // throttle / tilt axis
const HKEYS = { ArrowLeft: -1, ArrowRight: 1, a: -1, d: 1 };       // steer / pan axis

addEventListener("keydown", (e) => {
  if (e.key === " ") { e.preventDefault(); softStop(); return; }

  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (key === "x") { e.preventDefault(); setEstop(true); return; }
  if (key === "c") { e.preventDefault(); apply("campan", 0); apply("camtilt", 0); apply("steer", 0); return; }

  const v = VKEYS[key];
  if (v !== undefined) {
    e.preventDefault();
    // Shift -> camera tilt; otherwise throttle. Both step and hold, so key-repeat
    // ramps the value and releasing leaves it where it is.
    step(e.shiftKey ? "camtilt" : "drive", v);
    return;
  }
  const h = HKEYS[key];
  if (h !== undefined) {
    e.preventDefault();
    if (e.shiftKey) step("campan", h);                      // camera pan: step + hold
    else if (!e.repeat) apply("steer", h * controls.steer.keyVal); // steer: hold while pressed
    return;
  }
});

// Only steering springs back on release; throttle and camera hold their position.
// Spring regardless of Shift state so steering can never stick if Shift is pressed
// or released mid-hold (releasing a Shift+Left pan just re-centres already-0 steer).
addEventListener("keyup", (e) => {
  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (key in HKEYS) apply("steer", 0);
});

// Losing focus/visibility must stop the car (C-003): drop keep-alives and zero the
// throttle (which now holds) and steering.
function coast() { apply("drive", 0); apply("steer", 0); held = {}; }
addEventListener("blur", coast);
document.addEventListener("visibilitychange", () => { if (document.hidden) coast(); });

// Telemetry over WebSocket.
function handleTelemetry(ev) {
  const d = JSON.parse(ev.data);
  const set = (id, v) => { const e = el(id); if (v !== undefined) e.textContent = v; };
  if (d.cap === "battery") set("battery", d.volts?.toFixed?.(2));
  if (d.cap === "distance") {
    set("distance", d.cm);
    // Obstacle within the stop zone: kill the cruise throttle so it doesn't keep
    // driving into it. The component refuses forward until clear; reverse is fine.
    if (typeof d.cm === "number" && d.cm >= 0 && d.cm < 5 && +el("throttle").value > 0) {
      apply("drive", 0);
      setStatus("obstacle <5cm — throttle zeroed; reverse to back away", true);
    }
  }
  if (d.cap === "grayscale") set("grayscale", (d.adc || []).join(","));
  if (d.cap === "line") set("line", (d.line || []).join(","));
  if (d.cap === "cliff") {
    const e = el("cliff"); e.textContent = d.cliff; e.className = d.cliff ? "alarm" : "";
    // Cliff detected: kill the cruise throttle so the held forward value doesn't
    // keep driving into the edge. Only zero forward — leave reverse so the
    // operator can back away (the component refuses forward until clear anyway).
    if (d.cliff && +el("throttle").value > 0) {
      apply("drive", 0);
      setStatus("cliff detected — throttle zeroed; reverse to back away", true);
    }
  }
  if (d.cap === "sysinfo") set("sysinfo", d.fw);
}

// Reload the MJPEG feed (cache-busted) so it reconnects to a fresh stream after
// the robot restarts.
function reloadFeed() {
  const f = el("feed");
  if (f) f.src = "/stream/front?t=" + Date.now();
}
el("feed")?.addEventListener("error", () => setTimeout(reloadFeed, 1000));

// Auto-reconnecting telemetry socket: a robot restart or network blip recovers
// without a page refresh. Backoff grows to 5s and resets on a successful open.
let ws, wsBackoff = 500;
function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = handleTelemetry;
  ws.onopen = () => { wsBackoff = 500; setStatus(""); reloadFeed(); };
  ws.onclose = () => {
    setStatus("disconnected — reconnecting…", true);
    setTimeout(connectWS, wsBackoff);
    wsBackoff = Math.min(wsBackoff * 2, 5000);
  };
  ws.onerror = () => { try { ws.close(); } catch (_) {} };
}
connectWS();
