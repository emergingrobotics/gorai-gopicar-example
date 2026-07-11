# Requirements: `gorai-picarx` Teleoperable Robot

**Version:** 1.1
**Date:** 2026-07-11
**Status:** Draft
**Robot ID:** `picarx`

> **Revision 1.1 (2026-07-11):** RTSP video output **removed** (R-123 withdrawn; video
> is now the two paths in-page MJPEG + NATS frames). Documented the build-tag-selected
> camera capture sources (`rpicam` for Pi CSI cameras, `v4l2` for USB/UVC) and the fact
> that the robot runs from its **locally built binary**, not the stock `gorai run`,
> because it contains custom components. See R-104, R-105, R-120, and §7.

> **North star: [VISION.md](VISION.md).** This document is the authoritative, detailed
> requirement set that implements the vision. Where this document and `VISION.md` disagree,
> the VISION is authoritative. The master design derived from these requirements lives in
> [`docs/DESIGN.md`](docs/DESIGN.md).

Requirements are numbered for traceability. Design elements and code cite these IDs.
`MUST`/`MUST NOT`/`SHOULD`/`MAY` carry RFC-2119 weight.

---

## 1. Scope

A single GoRAI binary running on the Raspberry Pi inside a SunFounder PiCar-X. It exposes
the car's sensors as NCP resources and its actuators as NCP tools over an embedded NATS
mesh, serves one embedded web control page (video + telemetry + drive controls), streams
live video two ways (in-page MJPEG, NATS frames), and exposes the NATS bus on the LAN for
other agents. Built from the `gorai-robot-template` layout; motion/sensors via the
[`gopicar`](../gopicar) driver; video via a GoRAI `camera` component reading the Pi camera.
Because the robot contains custom components, it runs from its **own compiled binary**
(`bin/picarx`), not the stock `gorai run` (see R-104).

### 1.1 In scope
- NCP capability surface for all `gopicar` sensors and actuators.
- Live camera video: in-page (MJPEG) and over NATS.
- Embedded web teleop UI: sliders **and** keyboard, with a video panel.
- LAN-exposed NATS bus with credentialed access.
- Safety enforcement at the capability nodes (clamp, watchdog, e-stop, cliff interlock).
- JetStream audit of commands and scalar telemetry.

### 1.2 Out of scope (this version)
- Autonomous behaviors (line-follow, obstacle-avoid) — the tools they would call exist,
  but no agent is shipped.
- WebRTC in-page video (MJPEG is the v1 in-page path).
- **RTSP output of any kind** (RTP/JPEG or H.264). Withdrawn in v1.1: it added a
  version-pinned dependency (`gortsplib`) for a path no in-scope consumer used, and is
  not natively browser-playable. External viewers consume the NATS frame stream instead.
- Composite Robot / runtime discovery (the switch exists but defaults off).
- Persisting raw video frames in the audit stream.

---

## 2. Functional Requirements

### 2.1 Robot definition & packaging
- **R-100** The robot MUST be defined by a `robot.json` (RDL v2) and run via `gorai run`,
  producing/using a single binary with embedded NATS — no external services.
- **R-101** The project MUST follow the `gorai-robot-template` layout (`main.go` with
  blank-import manifest, `robot.json`, `components/`, `services/`, `Makefile`).
- **R-102** Components and services MUST self-register via `init()` calling
  `registry.RegisterComponent` / the service registry; `main.go` blank imports MUST be the
  only manifest.
- **R-103** The binary MUST cross-compile to `linux/arm64` for the Raspberry Pi.
- **R-104** The robot MUST be run from the project's **own compiled binary** (`bin/picarx`),
  produced by `gorai build` / `make build`, **not** the stock `gorai run`. Rationale: the
  stock `gorai` binary contains only the framework's built-in components; the custom
  `picarx`, `picam`, and `teleop-ui` types self-register (R-102) only inside a binary
  compiled from *this* module. Running `gorai run robot.json` therefore fails with
  `type … not found in registry`. `make run` MUST build then exec `./bin/<name> run …`.
- **R-105** The hardware-facing capture backend MUST be selected at build time via a Go
  **build tag**, so host builds and tests use a hardware-free fake source and never link
  platform code. The build system MUST pass the tag through (`gorai build --tags <tags>`,
  surfaced as `make ... TAGS=<tags>`; default `rpicam`). See R-120 for the camera sources.

### 2.2 `picarx` capability component (wraps `gopicar`)
- **R-110** The component MUST wrap `gopicar` `pkg/picarx` and open the device once at
  startup with an operator-supplied calibration, closing it cleanly on shutdown.
- **R-111** It MUST register the following **resources** (snapshot on `…state`, stream on
  `…data`) under `gorai.picarx.…`:
  - **R-111.1** `sensor.battery` — volts (float), from `Battery`, streamed ≥ 1 Hz.
  - **R-111.2** `sensor.distance` — centimetres (float), from `Distance`; `-1` on no-echo;
    streamed ~10 Hz.
  - **R-111.3** `sensor.grayscale` — `[3]int` raw ADC (left/middle/right), from `Grayscale`.
  - **R-111.4** `sensor.line` — `[3]bool` from `LineStatus` against a configured reference.
  - **R-111.5** `sensor.cliff` — `bool` from `CliffStatus`; MUST also publish `…event` on a
    rising edge (cliff detected).
  - **R-111.6** `system.info` — firmware `major.minor.patch`, HAT model, I²C address, from
    `FirmwareVersion`/`HAT`/`Addr`; snapshot only.
- **R-112** It MUST register the following **tools** (`…command`, JSON-Schema-validated
  args) under `gorai.picarx.…`:
  - **R-112.1** `base.drive` — `throttle` (signed %, −100..100); `>0` `Forward`, `<0`
    `Backward`, within deadband ⇒ `Stop`. MUST use `Ramp` internally to avoid current spikes.
  - **R-112.2** `servo.steer` — `angle` deg (+right/−left), via `SetDir`.
  - **R-112.3** `servo.campan` — `angle` deg (+right/−left), via `SetCamPan`.
  - **R-112.4** `servo.camtilt` — `angle` deg (+up/−down), via `SetCamTilt`.
  - **R-112.5** `base.estop` — no args; immediate `Stop`, latches until explicitly cleared.
- **R-113** Every tool MUST return a structured reply: success, or a typed error
  (`out_of_range`, `mcu_unavailable`, `estop_latched`, …). It MUST NOT silently no-op.
- **R-114** All tool/resource schemas MUST be registered into the mesh `gorai-schemas`
  catalog so the full surface is discoverable without configuration. NOTE (see DESIGN §11):
  gorai does not enforce these schemas at runtime; argument validation and clamping are
  performed **in the component handler** (R-150), and schema registration serves discovery.

### 2.3 `camera` capability component (Pi camera)
- **R-120** A `camera` component (model `picam`) MUST capture the Pi camera, JPEG-encode
  frames, and publish them as the NCP resource `gorai.picarx.front.data` (concrete
  `gorai.<robot>.<capability>.<type>` form; see DESIGN §11 R3/R7). The capture backend is
  build-tag-selected (R-105) via a `sourceFactory` swapped in each source file's `init()`:
  - **R-120.1** `-tags rpicam` (**default** for this robot) MUST capture via the Raspberry
    Pi camera stack by driving `rpicam-vid --codec mjpeg` and splitting its MJPEG output
    into frames. This is REQUIRED for CSI cameras (e.g. the OV5647 PiCam) on a Raspberry
    Pi 5, where `/dev/video0` is the raw CFE (`rp1-cfe`) and cannot be streamed as a plain
    V4L2 device — `VIDIOC_STREAMON` returns `EINVAL` because capture must traverse the
    libcamera media pipeline (CFE → ISP debayer). See §7.
  - **R-120.2** `-tags v4l2` MUST capture a USB/UVC webcam via GoRAI's `camera/v4l2`
    component. It MUST NOT be used for Pi CSI cameras on Pi 5.
  - **R-120.3** The two tags are mutually exclusive; if both are set, `rpicam` MUST win
    (enforced by the `//go:build v4l2 && !rpicam` constraint) so exactly one `init()`
    installs the `sourceFactory`. A build with neither tag MUST fail loudly at start
    ("no camera source compiled in"), never silently no-op.
- **R-121** `camera.front` MUST expose metadata on `…camera.front.state` (resolution,
  encoding, frame rate) and MUST report `SupportsPTZ` truthfully (the gimbal is a separate
  `picarx` tool, not driven by this component).
- **R-122** The NATS frame stream MUST be rate- and resolution-limited (default target
  ~10–15 fps at a modest resolution) to protect Wi-Fi and CPU.
- **R-123** *(Withdrawn in v1.1.)* Previously required an embedded RTSP (RTP/JPEG) endpoint.
  Removed — see §1.2. External viewers consume the `gorai.picarx.front.data` NATS frame
  stream instead. The `rtsp` config block and `rtspServer` code have been deleted.
- **R-124** The single physical capture MUST feed both consumers (in-page MJPEG, NATS
  frames); the camera MUST NOT be opened more than once.

### 2.4 `teleop-ui` service (embedded web page)
- **R-130** The service MUST serve exactly one control page from assets embedded in the Go
  binary via `embed.FS`. It MUST NOT require any external web server, external files, or
  CDN-hosted assets.
- **R-131** The page MUST be served from within the single robot binary — the `teleop-ui`
  service runs its own `http.Server` on its configured `listen` address (no separate
  process). The built-in `dashboard` is disabled so `teleop-ui` is the sole web surface.
  (See DESIGN §11 R8: gorai services receive only the raw NATS connection, not the robot's
  dashboard, so the service owns its listener.)
- **R-132** The `teleop-ui` server MUST be the mesh client: it subscribes to the sensor
  resource streams and issues actuator tool calls over NATS. The browser MUST NOT be a NATS
  client and MUST NOT hold NATS credentials.
- **R-133** The page MUST display **all** `gopicar`-derived data live, without manual
  refresh: battery, distance, grayscale ×3, line ×3, cliff, and system info.
- **R-134** The page MUST display a live **video panel** fed by an MJPEG stream
  (`multipart/x-mixed-replace`) that the server bridges from the `camera.front` NATS frames.
- **R-135** The page MUST provide **slider/pointer controls** for throttle, steering,
  camera pan, and camera tilt, plus an always-visible **STOP** button.
  - **R-135.1** Throttle and steering sliders MUST spring-return to zero/centre on release.
  - **R-135.2** Camera pan/tilt sliders MUST hold position.
- **R-136** The page MUST provide **keyboard controls**, active alongside the sliders:
  - **R-136.1** `W`/`S` or `↑`/`↓` → throttle forward/back; release ⇒ throttle 0.
  - **R-136.2** `A`/`D` or `←`/`→` → steer right/left; release ⇒ centre.
  - **R-136.3** `Space` → e-stop.
  - **R-136.4** `I`/`K` → camera tilt up/down; `J`/`L` → camera pan left/right.
  - **R-136.5** `C` → centre steering and camera.
- **R-137** Sliders and keyboard MUST map to the **same** NCP tool calls; the two input
  modes MUST be indistinguishable at the `…command` subject.
- **R-138** Telemetry updates to the browser MUST be pushed (WebSocket), not polled.

### 2.5 NATS bus exposure
- **R-140** The NATS bus and the browser-serving HTTP listener MUST be reachable over the
  Pi's Wi-Fi LAN, not only `localhost`. NOTE (verified, DESIGN §11): gorai derives the
  embedded server's bind *and* its own client dial from the single `nats.url`; `0.0.0.0`
  breaks the local dial, so LAN reach is achieved by setting `nats.url` to the Pi's routable
  address/hostname, or by using an external `nats-server` (which R-141 requires anyway). The
  `teleop-ui` HTTP listener binds `0.0.0.0` directly (its own `net.Listen`), so it is
  LAN-reachable without this caveat.
- **R-141** Joining the mesh MUST require valid credentials for the `picarx` NATS account,
  implemented as **NKey/JWT accounts** (signed credentials); the credential set is the
  robot's authorization boundary and maps to the Composite Robot model. A relaxed/no-auth
  mode MAY be offered for trusted-bench testing but MUST NOT be the deployed default.
- **R-142** With the exposed bus, a second machine on the LAN MUST be able to
  (a) list the `picarx` and `camera` capabilities via `gorai mesh services`/`schemas`, and
  (b) subscribe to `gorai.picarx.>` to observe the command/telemetry stream.
- **R-143** Runtime discovery / composition MUST default off (`discovery.enabled = false`);
  the robot MUST function fully as a self-contained base robot.

### 2.6 Safety (enforced at the capability node, per VISION §Safety)
- **R-150** Every actuator argument MUST be clamped to its physical/mechanical limit in the
  component handler before reaching a PWM channel, regardless of the wire value.
- **R-151** `base.drive` MUST implement a **watchdog/deadman**: it MUST require a fresh
  drive command within a configurable window (default 500 ms). On expiry the component MUST
  command `Stop` autonomously.
- **R-152** The `teleop-ui` browser client MUST heartbeat while any drive/steer control is
  active (held key or engaged slider) so the watchdog is satisfied only while an operator is
  present; loss of the browser, tab visibility, or Wi-Fi MUST let the watchdog stop the car.
- **R-153** `base.estop` MUST stop the motors immediately and latch; drive commands MUST be
  rejected with `estop_latched` until an explicit clear.
- **R-154** A `sensor.cliff` rising edge MUST force `Stop` in the component and block
  further drive until the cliff condition clears; the event MUST still be published.
- **R-155** Command and scalar-telemetry traffic on `gorai.picarx.>` MUST be persisted to a
  JetStream stream for replay. Raw video frames MUST NOT be persisted to the audit stream.

---

## 3. Constraints (anti-requirements)

- **C-001** NEVER let the browser hold NATS credentials or open a NATS connection —
  *Verified By* teleop-ui client review + network capture — *Stress* open the page with the
  NATS port firewalled off; controls MUST still work via the server.
- **C-002** NEVER drive a motor or servo past its clamped limit — *Verified By* component
  unit tests over out-of-range inputs — *Stress* flood `base.drive` with `throttle=9999`.
- **C-003** NEVER keep the car moving after operator contact is lost — *Verified By*
  watchdog test — *Stress* kill the browser tab mid-drive; car MUST stop within the window.
- **C-004** NEVER honor a drive command while e-stop is latched or a cliff is detected —
  *Verified By* interlock tests — *Stress* assert cliff, then send `base.drive`.
- **C-005** NEVER persist raw video frames in the JetStream audit stream — *Verified By*
  stream config review — *Stress* run video for 10 min; audit stream size stays bounded.
- **C-006** NEVER open the physical camera more than once — *Verified By* camera component
  review — *Stress* start MJPEG and NATS consumers simultaneously.
- **C-007** NEVER require a second process/web server for the UI — *Verified By* single
  `gorai run` produces the full experience.

## 4. Invariants

- **I-001** At most one calibrated `picarx` device handle is open for the process lifetime —
  *Manifested By* component open/close test.
- **I-002** Any value written to hardware is within `[min, max]` for that actuator —
  *Manifested By* property test over random wire inputs.
- **I-003** A slider action and the equivalent keypress produce byte-identical
  `…command` payloads — *Manifested By* input-equivalence test.
- **I-004** Every registered tool/resource has a schema present in `gorai-schemas` while the
  robot is running — *Manifested By* mesh discovery test.
- **I-005** The camera is captured exactly once and fanned out to all consumers —
  *Manifested By* single-capture assertion under concurrent consumers.

## 5. Behavior (Given/When/Then at boundaries)

- **B-001** *Given* the robot is running and calibrated, *When* a client requests
  `sensor.battery.state`, *Then* it receives the current voltage as a typed reply.
- **B-002** *Given* throttle 50 held on the slider, *When* the operator releases it,
  *Then* the server sends `throttle=0` and the car ramps to stop.
- **B-003** *Given* an engaged drive and the browser tab is closed, *When* 500 ms elapse
  with no fresh command, *Then* the watchdog commands `Stop`.
- **B-004** *Given* e-stop pressed, *When* any `base.drive` arrives, *Then* it is rejected
  with `estop_latched` and the motors stay off until cleared.
- **B-005** *Given* the front wheels approach a table edge, *When* `sensor.cliff` goes true,
  *Then* the component stops the motors, publishes `…cliff.event`, and blocks drive.
- **B-006** *Given* a second LAN machine with valid credentials, *When* it runs
  `gorai mesh schemas`, *Then* it lists all `picarx` and `camera` capabilities.
- **B-007** *Given* the camera streaming, *When* a browser opens the page and a second LAN
  client subscribes to `gorai.picarx.front.data`, *Then* both receive live JPEG frames
  from one capture.

## 6. Traceability to VISION

| VISION section | Requirements |
|---|---|
| Capability Surface (resources/tools) | R-110…R-114 |
| Video: two delivery paths (in-page MJPEG + NATS) | R-120…R-124, R-134 |
| Web Control Page | R-130…R-138 |
| Exposing the NATS bus | R-140…R-143 |
| Safety at the capability | R-150…R-155, C-001…C-007 |
| Success criteria | B-001…B-007 |

---

## 7. Build & Run notes (implementation)

These notes record hard-won environment facts behind R-104, R-105, and R-120.

### 7.1 Run the compiled binary, not `gorai run` (R-104)
The stock `gorai` CLI runs a robot **in-process** from the components compiled into *that*
binary — it does not recompile the project. Since `picarx`, `picam`, and `teleop-ui` live
in this module, only a binary built from this module (`bin/picarx`) can register them.

```bash
make build            # gorai build robot.json -o bin/picarx --target <host> --tags rpicam
make run              # builds, then ./bin/picarx run robot.json
./bin/picarx run robot.json   # equivalent; note the "run" subcommand
```

`gorai run robot.json` fails with `type "picarx" model "picarx" not found in registry`.

### 7.2 Build tags select the camera source (R-105, R-120)
`gorai build` gained a `--tags` flag (added upstream in `gorai/cmd/gorai/commands/build.go`)
that is forwarded to `go build`. The Makefile exposes it as `TAGS` (default `rpicam`):

```bash
make build                 # -tags rpicam  (Pi CSI camera — default)
make build TAGS=v4l2       # -tags v4l2    (USB/UVC webcam)
```

- `components/camera/source_rpicam.go`  (`//go:build rpicam`) — spawns `rpicam-vid`.
- `components/camera/source_v4l2.go`    (`//go:build v4l2 && !rpicam`) — USB webcam.
- Neither tag → the default `sourceFactory` returns an error at start (fail loud).

### 7.3 Why CSI cameras need `rpicam` on a Raspberry Pi 5 (R-120.1)
The test rig's camera is an **OV5647** (PiCam) on a **Pi 5**. On Pi 5, `/dev/video0` is the
`rp1-cfe` CSI front-end that emits **raw Bayer** and requires the libcamera media pipeline
(CFE → `pispbe` ISP for debayer) driven via the V4L2 Request API. A plain V4L2 `open` +
`VIDIOC_STREAMON` (the `v4l2` webcam path) fails with **`EINVAL`**. `rpicam-vid` drives that
pipeline for us; the `rpicam` source shells out to it (`--codec mjpeg -o - -t 0 -n --flush`)
and splits the MJPEG byte stream on JPEG SOI/EOI (`0xFFD8`…`0xFFD9`) markers into frames.
Verified end-to-end: `GET /stream/front` delivers ~15 fps of valid JPEG.

Prerequisite on the Pi: `rpicam-apps` installed (`/usr/bin/rpicam-vid`) and the camera
enabled (`rpicam-hello --list-cameras` lists the sensor).
