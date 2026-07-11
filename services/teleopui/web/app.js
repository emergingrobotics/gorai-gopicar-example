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
  b.textContent = on ? "CLEAR E-STOP" : "STOP";
  b.classList.toggle("engaged", on);
}
el("stop").addEventListener("click", () => setEstop(!estopped));

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
//   C                        centre the camera (pan + tilt to 0)
//   Space                    stop motors (e-stop)
//   W/A/S/D                  mirror the arrows for throttle/steer (R-136)
const VKEYS = { ArrowUp: 1, ArrowDown: -1, w: 1, s: -1 };          // throttle / tilt axis
const HKEYS = { ArrowLeft: -1, ArrowRight: 1, a: -1, d: 1 };       // steer / pan axis

addEventListener("keydown", (e) => {
  if (e.key === " ") { e.preventDefault(); setEstop(true); return; }

  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (key === "c") { e.preventDefault(); apply("campan", 0); apply("camtilt", 0); return; }

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
const ws = new WebSocket(`ws://${location.host}/ws`);
ws.onmessage = (ev) => {
  const d = JSON.parse(ev.data);
  const set = (id, v) => { const e = el(id); if (v !== undefined) e.textContent = v; };
  if (d.cap === "battery") set("battery", d.volts?.toFixed?.(2));
  if (d.cap === "distance") set("distance", d.cm);
  if (d.cap === "grayscale") set("grayscale", (d.adc || []).join(","));
  if (d.cap === "line") set("line", (d.line || []).join(","));
  if (d.cap === "cliff") { const e = el("cliff"); e.textContent = d.cliff; e.className = d.cliff ? "alarm" : ""; }
  if (d.cap === "sysinfo") set("sysinfo", d.fw);
};
