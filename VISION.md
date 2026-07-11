# Vision: `gorai-picarx` — A Teleoperable GoRAI Robot on the PiCar-X Frame

**A SunFounder PiCar-X, turned into a first-class [GoRAI](../gorai/VISION.md) robot: every
sensor is a resource, every actuator is a tool, all of it live on a NATS mesh — and a
single embedded web page that reads all of it and drives the car.**

---

## The One-Sentence Thesis

> The PiCar-X stops being a Python demo script and becomes a **capability mesh**:
> its battery, grayscale array, ultrasonic range, line/cliff detectors, and **camera
> feed** are **NCP resources**; its steering, drive, and camera-gimbal servos are **NCP
> tools**; and a browser talking to that same mesh is just another agent — the *first*
> agent, and for now the primary one.

This robot is a worked example of the GoRAI north star ([`../gorai/VISION.md`](../gorai/VISION.md)):
**a robot is a set of capabilities on a message mesh, not a chassis.** Here the chassis
happens to be a single $70 hobby car — but nothing in the design assumes that. The web
UI never touches hardware; it reads resources and calls tools over NATS exactly the way
a planning agent or a fleet coordinator would. The car is teleoperable today and
agent-drivable tomorrow with zero changes to the capability layer.

---

## Why This Robot Exists

Two reasons, both in service of the platform.

1. **Prove the capability model on real, cheap, moving hardware.** The PiCar-X has the
   full vocabulary of a small mobile robot — a steered front axle, two rear drive motors,
   a pan/tilt camera mount, and a useful spread of sensors — driven entirely through the
   [`gopicar`](../gopicar) Go driver (no cgo, single static `linux/arm64` binary). If NCP
   is the right contract, it should make *this* robot pleasant to build and operate.

2. **Show that the human UI is not special.** A recurring temptation in robotics is to
   wire the control panel straight to the motors. GoRAI's bet is the opposite: the panel
   is a mesh client. This example makes that concrete — the web page issues the same
   `…command` requests and subscribes to the same `…data` streams that an autonomous agent
   would. If teleop works over NCP, autonomy works over NCP.

---

## What the Robot Is

A single GoRAI binary, built from the [`gorai-robot-template`](../gorai-robot-template)
layout, running `gorai run` on the Raspberry Pi inside the PiCar-X. It contains:

- an **embedded NATS server** (JetStream on) — the mesh, in-process, zero external services;
- a **`picarx` capability component** that wraps `gopicar` and registers the car's sensors
  as resources and its actuators (including the camera pan/tilt gimbal) as tools;
- a **`camera` capability component** that captures the Pi camera (V4L2/libcamera) and
  publishes video as an NCP resource, and also serves an embedded RTSP stream;
- a **`teleop-ui` service** that serves one self-contained control page, embedded in the
  Go binary — **no separate web server, no external files, no CDN**.

That is the whole deployable unit. Copy one binary to the Pi, run it, open a browser on
the same Wi-Fi. Everything else — discovery, audit, fan-out — is inherited from the mesh.

### Assumptions stated up front

- **The Pi is on Wi-Fi and reachable.** Control is over the LAN; there is no cloud hop.
- **Two distinct hardware sources.** Everything in the `gopicar` surface below — battery,
  grayscale, ultrasonic, line/cliff, and the steer/drive/**camera-gimbal servos** — comes
  through `gopicar`. The **camera image itself does not**: `gopicar` aims the gimbal but
  provides no video, so the picture is captured by a separate `camera` component reading
  the Pi camera directly (V4L2/libcamera). The gimbal (a `gopicar` tool) and the video (a
  `camera` resource) are paired in the UI — you aim where you look — but they are different
  capabilities from different code.
- **Base robot, discovery off by default.** This is a self-contained single-platform robot.
  The NATS bus is still exposed externally (below), so other agents and other GoRAI
  platforms *can* join — but runtime composition is an opt-in flag, not the default.

---

## The Capability Surface (NCP)

Every capability rides the GoRAI subject convention:

```
gorai.<robot_id>.<component>.<instance>.<suffix>
```

with `robot_id = picarx`. Snapshots are request/reply on `…state`; live streams are
pub/sub on `…data`; actions are request/reply on `…command` with JSON-Schema-validated
arguments; faults are pub/sub on `…event`. All schemas self-register into the mesh
`gorai-schemas` catalog, so any client discovers the full surface without configuration.

### Resources (sensors — read the world)

Sourced directly from the `gopicar` facade (`pkg/picarx`).

| Resource subject | `gopicar` source | Payload | Stream rate |
|---|---|---|---|
| `…sensor.battery.{state,data}` | `Battery(ctx)` | volts (float) | 1 Hz |
| `…sensor.distance.{state,data}` | `Distance(ctx, timeout)` | centimetres (float); `-1` on no-echo | 10 Hz |
| `…sensor.grayscale.{state,data}` | `Grayscale(ctx)` | `[3]int` raw ADC (left/middle/right) | 10 Hz |
| `…sensor.line.{state,data}` | `LineStatus(ctx, ref)` | `[3]bool` over/under reference | 10 Hz |
| `…sensor.cliff.{state,event}` | `CliffStatus(ctx, ref)` | `bool`; **also** fires `…event` on a rising edge | on change |
| `…system.info.state` | `FirmwareVersion`, `HAT`, `Addr` | firmware `major.minor.patch`, HAT model, I²C address | on request |
| `…camera.front.data` | Pi camera (V4L2/libcamera) | JPEG frame (see *Video* below) | ~10–15 fps, rate/res-limited |

The `camera.front` resource is sourced by the separate `camera` component, **not** by
`gopicar` — but it is a first-class mesh resource like any other, discoverable in
`gorai-schemas` and addressable by name. Its metadata (`…camera.front.state`) reports
resolution, encoding, and frame rate.

`cliff` is the one resource that is *also* an event: a detected cliff is a safety-relevant
threshold crossing, so it is pushed, not only polled — an agent (or the safety logic)
should never have to be mid-poll to learn the floor disappeared.

### Tools (actuators — change the world)

| Tool subject | `gopicar` call | Arguments (validated) | Clamp / limit |
|---|---|---|---|
| `…base.drive.command` | `Forward` / `Backward` / `Stop` | `throttle` −100..100 (signed %) | clamped to ±100; below deadband ⇒ `Stop` |
| `…servo.steer.command` | `SetDir(deg)` | `angle` degrees, `+`right/`−`left | clamped to the frame's mechanical steer limit |
| `…servo.campan.command` | `SetCamPan(deg)` | `angle` degrees, `+`right/`−`left | clamped to pan limit |
| `…servo.camtilt.command` | `SetCamTilt(deg)` | `angle` degrees, `+`up/`−`down | clamped to tilt limit |
| `…base.estop.command` | `Stop` | none | immediate; latches until cleared |

The PiCar-X steers with the **front servo** and drives both rear motors together
(`Forward`/`Backward` set both), so mobility is modelled as **one `base.drive` throttle +
one `servo.steer` angle**, not left/right differential. This matches the hardware and
keeps the control surface honest.

`Ramp` (smooth throttle transitions) is used *inside* the `base.drive` handler to avoid
current spikes; it is an implementation detail of the tool, not a separate wire command.

---

## Video: Two Delivery Paths, One Capability

The `camera` component captures the Pi camera once and offers the feed two ways, so both
the mesh and conventional video tools can consume it. Which you use is a
consumer-by-consumer choice; the camera captures a single time regardless.

- **Over NATS (`gorai.picarx.camera.front.data`).** JPEG frames published as an NCP
  resource — discoverable, fan-out (the teleop server, an agent, and a recorder all
  subscribe to the same stream), and reachable anywhere the mesh reaches. The **in-page
  video panel is fed from this same stream**: the `teleop-ui` server re-serves it to the
  browser as MJPEG over HTTP. One capture, one NATS frame stream; the browser just gets an
  MJPEG view of it.
- **RTSP (RTP/JPEG), from an embedded server in the same binary.** A standards-based
  real-time stream for external viewers (VLC, `ffmpeg`, an NVR) and for anything that
  speaks RTSP. It reuses the *same* JPEG frames (RTP/JPEG, RFC 2435) through a pure-Go RTSP
  server — no extra encoder, no second capture. The server is embedded in the GoRAI binary,
  so it is a *stream endpoint*, not a separate web application, and the "one binary, no
  separate web server" promise holds. RTSP is not natively playable in a browser, which is
  exactly why the in-page panel uses the MJPEG-over-HTTP path instead. (Hardware-encoded
  H.264 over RTSP is a future bandwidth optimization — see *Future*.)

**Honest tradeoff — do not persist raw video in the audit stream.** Full-rate video is far
heavier than the command/telemetry traffic. The `…camera.front.data` frame stream is
plain pub/sub (rate- and resolution-limited, e.g. ~10–15 fps at a modest resolution) and
is **excluded from JetStream persistence**; the RTSP endpoint carries the same frames for
anyone who wants a standards-based feed. The audit stream keeps every *command* and every
*scalar reading* — it does not keep every frame. This keeps the "audit everything"
principle meaningful (you can replay what the robot *did*) without turning the Pi's SD card
into a DVR.

---

## Safety Lives at the Capability, Never at the Client

Per the GoRAI vision, the browser is never trusted to be safe. Every constraint is
enforced in the `picarx` component handler — the code actually touching the Robot HAT:

- **Clamp every value** (`throttle`, all servo angles) before it reaches a PWM channel,
  regardless of what arrived on the wire.
- **Teleop watchdog / deadman.** The `base.drive` tool requires a fresh command within a
  short window (e.g. 500 ms). The web UI heartbeats while a control is held or a key is
  down; if the browser closes, the tab is hidden, or Wi-Fi drops, commands stop arriving
  and the robot **coasts to `Stop` on its own.** A teleop car that keeps driving after it
  loses its operator is a bug, not a feature.
- **E-stop is a first-class tool and a first-class key.** `…base.estop.command` latches
  the motors off; the spacebar (and an always-visible STOP button) maps to it.
- **Cliff interlock.** A `sensor.cliff` rising edge forces `Stop` in the component before
  any further drive command is honoured — the planner/operator is informed via the event,
  but the wheels have already stopped.
- **Structured errors.** "throttle out of range", "steer angle clamped", "MCU not
  responding" come back as typed replies an operator or agent can reason about — never a
  silent no-op.
- **Audit by default.** JetStream persists `gorai.picarx.>`, so every command and every
  reading is replayable: what was driven, when, and what the sensors saw. Autonomy without
  replay is folklore.

---

## The Web Control Page

One page, embedded in the binary via Go's `embed.FS` and served by the `teleop-ui`
service over the robot's existing HTTP listener. The **`teleop-ui` server is the mesh
client** — it subscribes to the sensor resource streams and issues actuator tool calls on
`gorai.picarx.>` exactly as any other agent would. The browser is its thin front-end,
reaching the server over HTTP, a WebSocket telemetry push, and an MJPEG video stream —
GoRAI's existing dashboard pattern (server-side NATS, `coder/websocket` to the browser,
NATS-frames-to-MJPEG for video). The browser holds no NATS credentials; the mesh boundary
stays inside the binary, and a slider move or keypress becomes an NCP tool call issued by
the server, identical to what an autonomous agent would send.

**Layout — read everything, drive everything:**

- **Video panel** (top, live): an MJPEG stream in a plain `<img>` — the `teleop-ui` server
  bridges the camera's `…camera.front.data` NATS frames to `multipart/x-mixed-replace` over
  HTTP (the same NATS-frames-to-MJPEG bridge GoRAI's dashboard already uses), so the feed is
  browser-native with no plugin or WebRTC. The camera pan/tilt sliders/keys sit alongside
  it, so aiming the gimbal and watching the feed are one motion. A "connecting…"/"no feed"
  placeholder shows if frames stop.
- **Telemetry panel** (live, from `…data` streams): battery volts with a low-voltage
  warning, ultrasonic distance (with a proximity bar), the three grayscale values, the
  three line-sensor booleans, cliff state, and a system-info footer (firmware, HAT, I²C
  address). Values update as messages arrive — no page polling.
- **Drive controls:**
  - a **throttle slider** (−100..100, spring-return to 0 on release),
  - a **steering slider** (left..right, spring-return to centre),
  - **camera pan** and **camera tilt** sliders (these hold position),
  - a large, always-visible **STOP** button.
- **Keyboard control** (for anyone who'd rather drive than drag):
  - **W/S** or **↑/↓** — throttle forward/back; release ⇒ throttle 0,
  - **A/D** or **←/→** — steer left/right; release ⇒ centre,
  - **Space** — e-stop,
  - **I/K/J/L** — camera tilt up/down, pan left/right,
  - **C** — centre steering and camera.
  Held keys re-issue commands on an interval to satisfy the watchdog; `keyup` releases.
- The sliders and the keyboard are two front-ends to the *same* tool calls; moving a
  slider and pressing a key are indistinguishable at the `…command` subject.

**Why embedded, why a mesh client:** embedding the page (no separate web server, no build
step, no external assets) keeps the "one binary on the Pi" promise intact. Making it a
mesh client keeps the architecture honest — the day an autonomous agent takes the wheel,
the human panel keeps working alongside it, because both are just subscribers and callers
on `gorai.picarx.>`.

---

## Exposing the NATS Bus Externally

The embedded NATS server binds so it is reachable from the Wi-Fi LAN, not only
`localhost`:

- **The NATS client port (`4222`)** listens on the Pi's LAN interface, so external agents
  (a laptop, a fleet coordinator, another GoRAI platform) can join the mesh directly. The
  **browser** connects only to the `teleop-ui` HTTP listener (page + telemetry WebSocket +
  MJPEG) — not to NATS — so exposing the bus is about *agents*, not the web page.
- The **web page remains the primary control surface**; the exposed bus is what makes the
  robot a *mesh participant* rather than a closed appliance — it is how a second agent
  joins, how the audit stream is tapped, and how this car could later become one capability
  of a larger Composite Robot.
- **Authorization is the boundary.** Following the GoRAI model, joining the mesh requires
  valid credentials for this robot's NATS account; the credential set *is* the robot's
  edge. (For bench testing on a trusted LAN this may be relaxed; the vision assumes it is
  tightened before the car leaves the bench.)

---

## Architecture at a Glance

```
Browser (video + sliders + keys)         Other agents / GoRAI platforms / VLC
        │  HTTP + WebSocket + MJPEG                   │  NCP over NATS (LAN) + RTSP
        ▼                                            ▼
┌───────────────────────────────────────────────────────────────┐
│  gorai binary on the Raspberry Pi  (`gorai run robot.json`)     │
│                                                                 │
│  embedded NATS (JetStream: audit + discovery KV)  ·  embedded RTSP server
│      │                                                          │
│      ├─ teleop-ui service   → serves the embedded control page  │
│      │                         (embed.FS, no external server)   │
│      │                                                          │
│      ├─ camera component     → …camera.front.data (JPEG/NATS)   │
│      │        │                 + RTSP (RTP/JPEG); one capture  │
│      │        ▼                                                 │
│      │   Pi camera (V4L2/libcamera)                             │
│      │                                                          │
│      └─ picarx component     → resources (sensors) + tools      │
│              │                  (actuators + gimbal); safety    │
│              ▼                                                  │
│         gopicar / pkg/picarx  → Robot HAT MCU over I²C + GPIO   │
└───────────────────────────────────────────────────────────────┘
        │
        ▼
  PiCar-X hardware: steer servo, 2 rear motors, cam pan/tilt gimbal,
  battery ADC, grayscale ADC, HC-SR04 ultrasonic  +  Pi camera
```

Each layer has one job: **the page (and agents) reason and command; NATS distributes and
audits; the `picarx` and `camera` components enforce safety/limits and drive hardware.**

---

## Adherence to GoRAI Design Principles

| GoRAI principle | How this robot honours it |
|---|---|
| Capabilities over NATS (NCP) | Sensors are `…state`/`…data` resources; actuators are `…command` tools; cliff faults are `…event`. |
| A robot is a logical scope, not a chassis | `robot_id = picarx` is a scope; the bus is exposed so the car can later join a Composite Robot. Discovery is an opt-in flag. |
| Single binary deployment | One `gorai run` binary, embedded NATS, no containers/K8s/external services. |
| NATS-based messaging | Sensors, actuators, agents, and the `teleop-ui` server all communicate via the embedded mesh; the browser reaches the mesh only through the server's HTTP/WebSocket/MJPEG bridge, holding no NATS credentials. |
| Caddy-model components | `picarx` self-registers via `init()` → `registry.RegisterComponent`; a blank import in `main.go` is the manifest. |
| Go-first, pragmatic | Go core throughout; the UI is embedded assets, not a separate JS service. The motion/sensor path is pure Go (`gopicar` is cgo-free); the `camera` component may use a cgo V4L2/libcamera driver, which per GoRAI's language strategy is the accepted place for cgo (camera drivers) — kept isolated in its own component. |
| Safety at the capability | Clamps, watchdog/deadman, latching e-stop, cliff interlock, structured errors — all in the component, none in the client. |
| Audit / replay first-class | JetStream persists `gorai.picarx.>`; every command and reading is replayable. |

---

## Success Criteria

The vision is realised when, on the actual PiCar-X:

1. `gorai run robot.json` starts one binary; no other process is required.
2. A browser on the same Wi-Fi opens the embedded page and shows **live** video plus
   battery, distance, grayscale, line, and cliff data with no manual refresh.
3. Sliders **and** keyboard both steer the car, set speed and direction, and aim the
   camera gimbal — via the same NCP tool calls — and the video pans/tilts with the gimbal.
4. The camera feed is consumable **both** in-page (frames over NATS) **and** by an external
   RTSP client (e.g. VLC opening `rtsp://<pi>:8554/front`) from the same running binary.
5. Releasing all controls, closing the tab, or dropping Wi-Fi **stops the car** within the
   watchdog window; spacebar/STOP e-stops immediately.
6. `gorai mesh services` / `gorai mesh schemas` from a second machine on the LAN lists the
   `picarx` and `camera` capabilities — proving the bus is genuinely exposed and
   self-describing.
7. `nats sub 'gorai.picarx.>'` from that second machine shows the full command/telemetry
   stream — proving audit and fan-out are real, not claimed.

---

## Future (explicitly out of scope for this draft)

- **Hardware-encoded H.264 over RTSP** (Pi libcamera / V4L2 M2M) as a bandwidth
  optimization over the RTP/JPEG feed, added as a second camera output without disturbing
  the single-capture NATS/MJPEG paths.
- **WebRTC in-page video** as an upgrade to the MJPEG panel, for lower latency and H.264
  in the browser once the H.264 output above exists — without leaving the server-as-mesh-
  client model behind.
- **Vision on the frame stream:** an agent subscribing to `…camera.front.data` for
  line-follow, obstacle-avoid, or object-tracking that then calls the exact same
  `base.drive` / `servo.steer` / gimbal tools the human uses.
- **Composite Robot membership:** flip `discovery.enabled` on and let this car contribute
  its mobility, sensors, and camera to a larger, multi-platform robot.

These are deliberately deferred so the first build proves the core contract — teleop and
live video over NCP, safely, from one embedded binary — before anything is layered on top.

---

*Draft target: build the whole robot here against the `gopicar` fakes and the GoRAI mesh,
then finish integration testing on the physical PiCar-X.*
