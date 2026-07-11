# Design: `gorai-picarx` Teleoperable Robot

**Version:** 1.0
**Date:** 2026-07-10
**Status:** Draft — open decisions resolved (§10); ready for implementation-plan review.

> Derived from [`../REQUIREMENTS.md`](../REQUIREMENTS.md), which implements
> [`../VISION.md`](../VISION.md). Requirements are authoritative; if this design and the
> requirements disagree, fix this design. Design elements cite requirement IDs (`R-nnn`),
> constraints (`C-nnn`), and invariants (`I-nnn`).

---

## 1. Architecture

Three units in one binary, plus the mesh they share.

```
┌──────────────────────── gorai binary (single process) ────────────────────────┐
│                                                                                │
│  embedded NATS (embeddednats)  ── JetStream: audit + gorai-{services,          │
│        │  LAN listener :4222        channels,schemas} KV                        │
│        │                                                                        │
│  ┌─────┴───────────┐   ┌───────────────────┐   ┌───────────────────────────┐   │
│  │ picarx component│   │ camera component  │   │ teleop-ui service          │   │
│  │ (wraps gopicar) │   │ (Pi camera)       │   │ (embedded web + bridges)   │   │
│  │                 │   │                   │   │                            │   │
│  │ resources:      │   │ …camera.front.data│   │ HTTP :dashboard.listen     │   │
│  │  battery,       │   │   (JPEG/NATS)     │   │  ├─ GET /            page   │   │
│  │  distance,      │   │ …camera.front.state│  │  ├─ GET /stream/front MJPEG│   │
│  │  grayscale,     │   │ RTSP :8554/front  │   │  ├─ WS  /ws     telemetry  │   │
│  │  line, cliff,   │   │ RTP/JPEG (D-1)    │   │  └─ POST/WS control→tools  │   │
│  │  system.info    │   │                   │   │                            │   │
│  │ tools:          │   │ single capture,   │   │ mesh client: subscribes    │   │
│  │  base.drive,    │   │ fan-out (I-005)   │   │ resources, calls tools     │   │
│  │  servo.{steer,  │   │                   │   │ (R-132)                    │   │
│  │  campan,camtilt}│   └───────────────────┘   └───────────────────────────┘   │
│  │  base.estop     │                                                            │
│  │ SAFETY: clamp,  │                                                            │
│  │ watchdog,estop, │                                                            │
│  │ cliff interlock │                                                            │
│  └───────┬─────────┘                                                            │
│          ▼ gopicar/pkg/picarx → Robot HAT MCU (I²C) + GPIO                       │
└────────────────────────────────────────────────────────────────────────────────┘
```

### 1.1 Modules & responsibilities (single responsibility each)

| Module | Path (this repo) | Responsibility | Reqs |
|---|---|---|---|
| `picarx` component | `components/picarx/` | Wrap `gopicar`; register resources+tools; enforce safety | R-110…R-114, R-150…R-155 |
| `camera` component | reuse `gorai/components/camera/v4l2` (config in `robot.json`) + local RTSP piece | Capture Pi camera, publish JPEG frames, serve RTSP | R-120…R-124 |
| `teleop-ui` service | `services/teleop-ui/` | Serve embedded page; bridge NATS↔browser (WS + MJPEG); translate UI events to tool calls | R-130…R-138 |
| Robot definition | `robot.json`, `main.go`, `Makefile` | RDL v2, blank-import manifest, build/deploy | R-100…R-103 |

### 1.2 Boundaries & third-party assumptions
- **`gopicar` guarantees** (from its README/API): cgo-free, context-aware calls; `Open`
  requires a `Calibration`; `Distance` returns `-1` on no-echo; steering is a single front
  servo, drive is both rear motors together (no differential). The `picarx` component owns
  the single `*picarx.PiCarX` handle (I-001).
- **GoRAI core guarantees**: `registry.RegisterComponent` self-registration; `embeddednats`
  provides in-process NATS+JetStream; the `dashboard` package provides an embedded
  `http.Server`, `go:embed` static assets, a `coder/websocket` push hub, and
  `dashboard/cameras.StreamHandler` which "bridges NATS camera frames to HTTP MJPEG"
  (`multipart/x-mixed-replace`). The `teleop-ui` service reuses these patterns rather than
  reinventing them.
- **`camera/v4l2` reality (verified, §11):** captures the Pi camera and JPEG-encodes, and
  exposes `SetFrameCallback(fn)` — but it does **not** publish to NATS itself (no production
  caller wires the callback). Our `camera` component owns the capture and sets the callback
  to publish JPEG to `gorai.picarx.front.data` and to feed the RTSP muxer — one capture,
  fanned out (I-005, C-006).

---

## 2. Interfaces

### 2.1 NCP subject map (`robot_id = picarx`)

Convention (concrete, per gorai's `subjects.Builder` — see §11): `gorai.picarx.<capability>.<type>`
where `<type>` ∈ `{data,state,command,event}`. The idealized 5-part NCP names in VISION
(`sensor.battery`, `base.drive`, `servo.steer`) map to single-token capabilities here:
`battery, distance, grayscale, line, cliff, sysinfo, drive, steer, campan, camtilt, estop,
front` (camera). Subjects below use those concrete names.

All subjects are prefixed `gorai.picarx.`.

| Subject | Kind | Payload / args (JSON) | Req |
|---|---|---|---|
| `battery.state` / `battery.data` | resource | `{ "volts": number }` | R-111.1 |
| `distance.state` / `distance.data` | resource | `{ "cm": number }` (`-1` = no echo) | R-111.2 |
| `grayscale.state` / `grayscale.data` | resource | `{ "adc": [int,int,int] }` | R-111.3 |
| `line.state` / `line.data` | resource | `{ "line": [bool,bool,bool] }` | R-111.4 |
| `cliff.state` / `cliff.data` | resource | `{ "cliff": bool }` | R-111.5 |
| `cliff.event` | event | `{ "cliff": true, "ts": rfc3339 }` (rising edge) | R-111.5, R-154 |
| `sysinfo.state` | resource | `{ "fw":"M.m.p", "hat":str, "addr":int }` | R-111.6 |
| `front.data` | resource | JPEG bytes (raw `msg.Data`) | R-120 |
| `front.state` | resource | `{ "w":int,"h":int,"enc":"jpeg","fps":num,"ptz":bool }` | R-121 |
| `drive.command` | tool | `{ "throttle": number }` −100..100 | R-112.1 |
| `steer.command` | tool | `{ "angle": number }` deg +right/−left | R-112.2 |
| `campan.command` | tool | `{ "angle": number }` deg +right/−left | R-112.3 |
| `camtilt.command` | tool | `{ "angle": number }` deg +up/−down | R-112.4 |
| `estop.command` | tool | `{ "clear": bool }` (omit/false = engage) | R-112.5, R-153 |

`front.data` is a single-token capability name so gorai's `dashboard/cameras.StreamHandler`
(`ComponentData("front")` → `gorai.picarx.front.data`) can bridge it unmodified if reused.

**Serve mechanics (verified, §11):** the `picarx` component implements the command *server*
itself — in its `Start(ctx)` it `nc.Subscribe`s each `*.command` subject, JSON-unmarshals
`map[string]any`, dispatches to the handler, and replies with `msg.Respond(jsonBytes)`
(the request/reply form gorai's `pkg/proxy` clients already speak). Resource `*.data`
streams are published with `nc.Publish(subject, jsonBytes)`; `*.state` is answered the same
request/reply way as commands. Tool replies: `{ "ok": true }` or
`{ "ok": false, "error": "<code>", "msg": str }` where `<code>` ∈
`out_of_range | mcu_unavailable | estop_latched | cliff_blocked` (R-113). JSON-schema
validation is **not** runtime-enforced by gorai; handlers validate and clamp in-code
(safety at the node, R-150), and schemas are registered into `gorai-schemas` for discovery.

### 2.2 HTTP / stream endpoints (teleop-ui, on its own listen address)

| Method | Path | Purpose | Req |
|---|---|---|---|
| GET | `/` | The embedded control page (single HTML from `embed.FS`) | R-130, R-131 |
| GET | `/static/*` | Embedded CSS/JS (no CDN) | R-130 |
| GET | `/stream/front` | MJPEG `multipart/x-mixed-replace`, bridged from `…camera.front.data` | R-134 |
| GET | `/ws` | WebSocket: server→browser telemetry push; browser→server control events | R-132, R-138 |

Control events over `/ws` (browser→server) are translated by the server into the tool
calls in §2.1; the browser never addresses NATS (C-001). Event shape:
`{ "t":"drive|steer|campan|camtilt|estop|centre", "v": number }`.

### 2.3 RTSP endpoint
`rtsp://<pi-lan-ip>:8554/front` — external viewers/recorders (R-123). **RTP/JPEG (RFC
2435)** via a pure-Go RTSP server (e.g. `bluenviron/gortsplib`), fed by the *same* JPEG
frames as the NATS/MJPEG paths (one capture, I-005/C-006; no encoder). Decision D-1 (§10).

---

## 3. State

| Entity | Kind | Model | Notes |
|---|---|---|---|
| `PiCarX` handle | ephemeral | opened once, closed on shutdown | I-001 |
| e-stop latch | ephemeral, in `picarx` | boolean state machine | R-153 |
| cliff interlock | ephemeral, in `picarx` | derived from `sensor.cliff`; blocks drive while true | R-154 |
| drive watchdog | ephemeral, in `picarx` | timer reset on each `base.drive`; expiry ⇒ `Stop` | R-151 |
| latest camera frame | cached, in `camera` | last JPEG for `Image()` / late subscribers | R-120 |
| audit log | persistent (JetStream) | commands + scalar telemetry on `gorai.picarx.>`; **not** video (C-005) | R-155 |
| calibration | config input | operator-supplied `picarx.Calibration`; storage-agnostic | R-110 |

### 3.1 Drive-safety state machine (in the `picarx` component)

```
        ┌─────────── base.estop(engage) / cliff rising ───────────┐
        ▼                                                          │
   [STOPPED] ──base.drive(v≠0), not blocked──▶ [DRIVING] ──watchdog expiry──▶ [STOPPED]
        ▲                                          │                              
        │                                          ├─ base.drive(v≈0) ──▶ [STOPPED]
        └──────── base.estop(clear), cliff cleared ─┘  (drive rejected while blocked)
```

Transitions to `STOPPED` always write throttle 0 to hardware first, then update state
(I-002 holds through every transition). Watchdog window and drive deadband are RDL params.

### 3.2 Initialization order
`embeddednats` up → NATS connected → components created in dependency order (topo-sorted):
`picarx` opens the device and (in its `Start`) subscribes command subjects + starts sensor
publishers; `camera` opens capture and starts publishing frames → then **services** run in
config order (services are created after all components, so `teleop-ui` can rely on the
mesh being live). `teleop-ui`'s `Start` opens its own HTTP listener and connects its
handlers as a mesh client. It depends on the mesh, not on the other components existing; if
a capability is absent the page shows placeholders and its controls no-op with an error
toast.

---

## 4. Safety design (maps C-001…C-007, R-150…R-155)

- **Clamp (R-150, I-002):** each tool handler clamps to configured limits
  (`steer_max_deg`, `campan_max_deg`, `camtilt_max_deg`, throttle ±100) before any
  `gopicar` call. Out-of-range ⇒ clamp **and** return `out_of_range` so the caller learns.
- **Watchdog (R-151/152, C-003):** a `time.Timer` reset on each `base.drive`; on expiry the
  component calls `Stop`. The browser sends a keep-alive control event on an interval while
  a drive/steer input is active; ceasing input (release, tab hidden via
  `visibilitychange`, socket close) stops the keep-alive, so the watchdog fires.
- **E-stop (R-153, C-004):** `base.estop` sets a latch and calls `Stop`; `base.drive`
  returns `estop_latched` until `{"clear":true}`.
- **Cliff interlock (R-154, C-004):** the component subscribes to its own cliff reading;
  rising edge ⇒ `Stop` + publish `…cliff.event` + set `cliff_blocked`; `base.drive` returns
  `cliff_blocked` until cleared.
- **Audit (R-155, C-005):** JetStream stream on `gorai.picarx.>` **excluding** the
  `…camera.front.data` subject (frames go to plain pub/sub, not the stream).

All enforcement lives in the `picarx` component handlers — never in `teleop-ui` or the
browser (VISION §Safety). `teleop-ui`'s own keep-alive is a *convenience* that helps the
watchdog; it is not the safety mechanism.

---

## 5. teleop-ui design

- **Assets:** one `index.html` + `app.js` + `style.css` under `services/teleop-ui/web/`,
  embedded with `//go:embed web/*` (R-130). Vanilla JS — no framework, no CDN.
- **Own HTTP server (verified, §11):** because a *service* receives only the raw
  `*nats.Conn` (the robot builds and owns the `dashboard`; it is not handed to services),
  `teleop-ui` runs its **own** `http.Server` on its own listen address (RDL attribute,
  default `0.0.0.0:8080`) using `net/http` (chi optional). The built-in `dashboard` is
  disabled in `robot.json` so there is one web surface.
- **Video panel:** `<img src="/stream/front">`. The MJPEG handler is a ~40-line bridge
  modeled on `dashboard/cameras.StreamHandler` but written against the raw `*nats.Conn`
  (which `StreamHandler` cannot take — it needs `*gorainats.Client`): subscribe to
  `gorai.picarx.front.data`, set `Content-Type: multipart/x-mixed-replace; boundary=frame`,
  write each JPEG with the boundary/`Content-Length` header, `Flush()`, drop frames when
  the client is slow (R-134).
- **Telemetry:** the server reuses `dashboard.NewWebSocketHub()` (a standalone type with no
  NATS coupling): on start it subscribes to the six sensor `*.data` subjects and calls
  `hub.BroadcastJSON(...)`; `/ws` is served by `hub.HandleWebSocket` (R-133, R-138). The
  browser→server control channel is a second, small WS (or POST) endpoint.
- **Controls → tools (R-135/136/137, I-003):** both the sliders (`input`/`pointerup`
  events) and the keyboard (`keydown`/`keyup`) call one JS function `sendControl(type, v)`
  that emits the §2.2 control event. The server maps each to exactly one tool call, so a
  slider and its keypress produce identical `…command` payloads.
- **Keymap:** per R-136.1…R-136.5.
- **Spring-return:** throttle/steer sliders reset to 0/centre on `pointerup`/`keyup` and
  send the zero/centre command (R-135.1); camera sliders hold (R-135.2).

**Reuse decision (D-2, refined by §11):** `teleop-ui` ships as a **sibling service in this
repo**. It reuses `dashboard.WebSocketHub` verbatim (standalone, no NATS coupling) and
mirrors `cameras.StreamHandler`'s MJPEG writer, but runs its own `http.Server` — because
services are not handed the robot's dashboard or its `*gorainats.Client`. No change to
gorai core.

---

## 6. RDL (`robot.json`) sketch

```json
{
  "version": "2",
  "robot": { "name": "picarx", "description": "Teleoperable PiCar-X GoRAI robot" },
  "nats": { "url": "nats://127.0.0.1:4222", "jetstream": true },
  "dashboard": { "enabled": false },
  "components": [
    { "type": "picarx", "name": "picarx",
      "attributes": { "calibration": "calibration.json",
        "steer_max_deg": 30, "campan_max_deg": 90, "camtilt_max_deg": 65,
        "watchdog_ms": 500, "grayscale_ref": [1000,1000,1000] } },
    { "type": "camera", "model": "picam", "name": "front",
      "depends_on": ["picarx"],
      "attributes": { "device": "/dev/video0", "width": 640, "height": 480, "fps": 15,
        "jpeg_quality": 70,
        "rtsp": { "enabled": true, "listen": ":8554", "path": "/front" } } }
  ],
  "services": [ { "type": "teleop-ui", "name": "teleop",
      "attributes": { "listen": "0.0.0.0:8080", "camera": "front" } } ],
  "discovery": { "enabled": false }
}
```
(The built-in `dashboard` is disabled — `teleop-ui` is the sole web surface, on its own
`listen`. Verified against gorai (`pkg/robot` `startEmbeddedNATS` → `parseNATSURL`):
`NATSConfig` has **no** `embedded`/`listen` field — the embedded server binds the host+port
parsed from `nats.url`, and `jetstream:true` enables the KV catalog + audit stream.
**Verified by running the binary:** gorai derives *both* the listen bind and the robot's own
client dial from this one URL, and `nats://0.0.0.0:4222` makes the local client fail with
`no servers available` (dialing `0.0.0.0` is not accepted). So the default is
`127.0.0.1:4222` (starts reliably). To satisfy R-140 (LAN reach) set `nats.url` to the Pi's
**routable LAN address/hostname** at deploy (e.g. `nats://raspberrypi.local:4222`) — that
binds the LAN interface *and* lets the local client connect. For LAN + auth (R-141), prefer
an external `nats-server` bound `0.0.0.0` with accounts and `nats.external:true`
(docs/NATS-AUTH.md Option A). The single-URL bind+dial coupling is a gorai-core limitation.
`discovery.enabled=false`
satisfies R-143. Credentials/authz per R-141 use **NKey/JWT accounts** (D-3, §10),
configured via signed creds referenced from the `nats` block / out-of-band creds files.)

---

## 7. Build, deploy, run

Per the template `Makefile` (R-101): `make validate`, `make run`, `make build`,
`make deploy DEPLOY_HOST=pi@…`, with `TARGET=linux/arm64` for the Pi (R-103). `main.go`
blank-imports `components/picarx`, the camera model, and `services/teleop-ui` (R-102).

---

## 8. Testing strategy

- **Component unit tests (hardware-free):** drive `picarx` against `gopicar`'s fakes
  (`internal/fake`) — clamp (C-002/I-002), watchdog (C-003/B-003), e-stop (C-004/B-004),
  cliff interlock (B-005), reply codes (R-113).
- **Input-equivalence test (I-003):** assert slider-event and keypress-event map to
  identical `…command` payloads.
- **Mesh discovery test (I-004/B-006):** boot the robot, assert all schemas present via the
  catalog.
- **Bridge tests:** MJPEG handler emits valid `multipart/x-mixed-replace`; `/ws` telemetry
  reflects resource updates.
- **Single-capture test (I-005/C-006/B-007):** three concurrent consumers, one capture.
- **Hardware integration (on the Pi):** gated behind a build tag as `gopicar` does
  (`-tags hardware`, actuator moves behind an env flag), covering VISION success criteria
  1–7. This is the "finish testing on the actual RPi" step.

## 9. Implementation phases

1. **M1 — `picarx` component:** resources + tools + safety, tested on fakes. (R-110…R-114,
   R-150…R-155)
2. **M2 — camera over NATS + MJPEG:** wire `camera/v4l2`, verify in-page video via the
   MJPEG bridge. (R-120…R-122, R-134)
3. **M3 — teleop-ui:** embedded page, sliders + keyboard + telemetry WS + STOP. (R-130…R-138)
4. **M4 — NATS exposure + audit:** LAN binding, credentials, JetStream audit. (R-140…R-143,
   R-155)
5. **M5 — RTSP:** add the RTP/JPEG RTSP endpoint (gortsplib), fed by the shared frames. (R-123)
6. **M6 — Pi integration:** hardware tests, calibration, success criteria.

---

## 10. Decision record (resolved 2026-07-10)

- **D-1 — RTSP encoding path.** *Decision:* **RTP/JPEG (RFC 2435)** via a pure-Go RTSP
  server (`bluenviron/gortsplib`), reusing the existing JPEG frames — no encoder, one
  capture. *Why:* keeps the single-capture/pure-Go story, ships fast, and is enough for LAN
  viewing; H.264 over RTSP was deferred to Future because it adds a hardware encoder
  (libcamera/V4L2 M2M) and build complexity not justified for v1. *Alternatives rejected:*
  hardware H.264 (best bandwidth, but cgo/libcamera dependency now), software H.264 (x264
  via cgo — CPU-heavy on a Pi). Folded into VISION §Video, R-123.
- **D-2 — teleop-ui vs. dashboard.** *Decision:* ship `teleop-ui` as a **sibling service in
  this repo**, reusing `dashboard`'s `Server`, `WebSocketHub`, and `cameras.StreamHandler`
  as libraries. *Why:* keeps this robot's drive controls and keymap out of gorai core
  (smaller blast radius) while still reusing proven embedded-web machinery. *Alternative
  rejected:* extending `gorai/pkg/dashboard` in-place (would push robot-specific teleop into
  core and give every robot drive controls). Folded into §1.1, §5.
- **D-3 — NATS authz.** *Decision:* **NKey/JWT accounts** (signed credentials) for the
  LAN-exposed bus. *Why:* the credential set becomes the robot's authorization boundary and
  maps directly to the Composite Robot model (R-141, VISION §Exposing the NATS Bus); a
  relaxed bench mode may be used during hardware bring-up but is never the deployed default.
  *Alternative rejected:* user/password+TLS (simpler, but weaker multi-tenant/composition
  story). Folded into §6, R-141.

*All decisions folded back into REQUIREMENTS.md and VISION.md (spec-first). Next step: the
implementation plan (per the writing-plans workflow), sequenced by the milestones in §9.*

---

## 11. Implementation reconciliation (verified against gorai source, 2026-07-10)

Facts confirmed by reading `/gorai-all/gorai` and `/gorai-all/gopicar`. These refine the
idealized descriptions above; where they differ, §11 is correct.

**Module paths.** gorai: `github.com/emergingrobotics/gorai`. gopicar:
`github.com/emergingrobotics/gopicar`; the driver is `.../gopicar/pkg/picarx`.

**R1 — No auto-wiring; the component is its own NATS server.** `pkg/robot` only calls a
component's constructor, optionally `Start(ctx)` (the `Startable` interface,
`robot.go:357`), and `Close(ctx)`. It does **not** map `Readings()`/`DoCommand` onto NATS.
A component grabs the raw `*nats.Conn` via `deps.Get("nats")` (asserted `*nats.Conn`) and a
`*slog.Logger` via `deps.Get("logger")`, then does its own pub/sub in `Start`. So `picarx`
must, in `Start`: `nc.Subscribe("gorai.picarx.<cap>.command", handler)` and reply with
`msg.Respond(json)`, and launch goroutines that `nc.Publish("gorai.picarx.<cap>.data", json)`.

**R2 — Component contract.** `registry.RegisterComponent(subtype, model string, ctor)` where
`ctor = func(ctx, deps registry.Dependencies, conf registry.Config) (any, error)`;
`registry.Config = map[string]any`. The returned object must implement `resource.Resource`:
`Name() resource.Name`, `Reconfigure(ctx, resource.Dependencies, resource.Config) error`,
`DoCommand(ctx, map[string]any) (map[string]any, error)`, `Close(ctx) error`. Note the
constructor takes `registry.Dependencies/Config` but `Reconfigure` takes the `resource.*`
variants — match both exactly. `resource.NewComponentName("gorai", subtype, name)` builds
the name; `conf["name"]` etc. are injected by the robot. JSON numeric attributes arrive as
`float64`.

**R3 — Subjects.** `subjects.NewBuilder(robotID)` yields `Component(cap, type) =
"gorai.<robotID>.<cap>.<type>"` with `type ∈ {data,state,command,event}` and helpers
`ComponentData/State/Command/Event`. There is no 5-part form; the concrete capability names
in §2.1 are single tokens. `robotID` is the RDL namespace/robot name.

**R4 — Command server pattern.** gorai's `pkg/proxy` clients issue commands as
`nc.RequestWithContext(cmdSubject, json.Marshal(map[string]any))` and read a JSON
`map[string]any` reply; the *server* side is not generic, so we implement it. (An
alternative, `nws.Wrap`, exposes reflected methods on a `<name>.rpc` subject — not used here
because it does not fit the `.command` request/reply convention or in-handler safety.)

**R5 — Publishing JSON.** The deps bag hands out the raw `*nats.Conn`, not the
`*gorainats.Client` wrapper. Use `nc.Publish(subj, jsonBytes)` for `.data`; for `.state`
and `.command` use request/reply (`nc.Subscribe` + `msg.Respond`). `pkg/pub.Publisher[T]` is
protobuf-typed and not used for our JSON payloads.

**R6 — Schemas are for discovery only.** `mesh.NewClient(nc)` + `RegisterSchema(ctx,
SchemaDescriptor)` writes into the `gorai-schemas` KV; `mesh.NewJSONSchema(name, version,
desc, def)` builds the descriptor. gorai does **not** validate command args against these at
runtime — handlers validate/clamp in code (R-150). Registering schemas satisfies R-114's
*discoverability* intent.

**R7 — Camera.** `components/camera/v4l2` (`camera/v4l2` model) captures + JPEG-encodes and
exposes `SetFrameCallback(func(jpeg []byte, ts time.Time, seq uint64, frameID string))`, but
nothing in the runtime calls it. Our `camera` component (model `picam`) wraps a capture
source, sets that callback to (a) `nc.Publish("gorai.picarx.front.data", jpeg)` and (b) push
into the RTSP muxer, and answers `front.state` via request/reply. Capture is opened once
(I-005). For hardware-free tests the capture source is an interface with a fake that emits
canned JPEG frames.

**R8 — Service contract.** `registry.RegisterService(subtype, model, ctor)` with the same
`Constructor` shape. Services are created **after** all components (so the mesh and
components are live) and run in config order (services are *not* topo-sorted; components are,
via `depends_on` + `pkg/robot/topo.go`). A service that implements `Startable` has `Start`
called by the robot. The service also gets only the raw `*nats.Conn` from deps — hence
`teleop-ui` builds its own `http.Server`.

**R9 — Reusable web pieces.** `dashboard.NewWebSocketHub()` → `*WebSocketHub` with
`Run(ctx)`, `HandleWebSocket(w,r)`, `Broadcast([]byte)`, `BroadcastJSON(any)` — standalone,
no NATS coupling, reusable directly. `dashboard/cameras.StreamHandler` needs a
`*gorainats.Client` (not the raw conn), so we mirror its ~40-line MJPEG writer against the
raw conn instead of importing it. The dashboard itself is constructed by `pkg/robot`
(`dashboard.New(cfg, robotCfg, WithNATS, WithSubjects, WithLogger)`) and is disabled here.

**R10 — gopicar API used** (all `func (p *PiCarX) …(ctx, …) …error`): `Open(ctx, Options)`,
`Close()`, `SetDir(deg)`, `SetCamPan(deg)`, `SetCamTilt(deg)`, `Forward(pct 0..100)`,
`Backward(pct 0..100)`, `Stop()`, `Ramp(from,to,dur)`, `Battery()→(float64)`,
`Grayscale()→([3]int L,M,R)`, `Distance(timeout)→(float64 cm)`, `LineStatus(ref [3]int)→
[3]bool`, `CliffStatus(ref [3]int)→bool`, `FirmwareVersion()→(maj,min,patch uint8)`,
`HAT()`, `Addr()→uint8`. `Options.Bus`/`Options.Chip` inject `internal/fake` bus/chip for
tests; `Options.MCU.DeviceTreeRoot = t.TempDir()`. Sign conventions: steer/pan `+`right,
tilt `+`up, all `0` centered. `MeasuredCalibration()` steer range is ±30°.

**Consequent spec adjustments:** R-114 now reads "registered for discovery; enforcement
in-handler"; R-131 web listener is `teleop-ui`'s own address (built-in dashboard disabled),
not `dashboard.listen`; camera model is `picam` (local), not `v4l2` directly.
