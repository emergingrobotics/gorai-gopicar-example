# gorai-picarx

A SunFounder **PiCar-X** turned into a first-class [GoRAI](https://github.com/emergingrobotics/gorai)
robot: every sensor is a NATS **resource**, every actuator is a NATS **tool**, and a single
embedded web page reads the telemetry and drives the car. The browser is just another agent
on the mesh — the same interface a planning agent or fleet coordinator would use — so the car
is teleoperable today and agent-drivable tomorrow with no changes to the capability layer.

Everything runs as **one process** on the Pi: an embedded NATS server, the `picarx` and
`camera` components, and the `teleop-ui` web service.

---

## Run & operate

### On the Raspberry Pi

Prerequisites: 64-bit Pi OS, the camera enabled (`/dev/video0` present), and `gopicar`
udev/permissions set up per the gopicar docs. You need the built binary plus `robot.json` and
`calibration.json` alongside it (see [Build](#build) / [Deploy](#deploy)).

```sh
./picarx run robot.json
```

That single command:
- starts the embedded NATS server (JetStream: audit + service/schema catalog),
- opens the PiCar-X hardware and the camera,
- serves the control page on `http://<pi>:8080/`.

Then open a browser **on the same network** at `http://<pi>:8080/`.

### Driving from the browser

| Input | Action |
|---|---|
| `↑` / `↓` (or throttle slider) | Drive forward / reverse — **holds** while held |
| `←` / `→` (or steer slider) | Steer right / left — spring-returns to center on release |
| `Shift`+`↑` / `↓` | Camera tilt up / down (holds position) |
| `Shift`+`←` / `→` | Camera pan (holds position) |
| `C` | Center camera + wheels |
| `Space` | Stop |
| `X` (or the **EMERGENCY STOP** button) | E-stop — latches; drive is refused until cleared |
| **QUIT ROBOT** button | Graceful shutdown of the whole robot (motors stopped, camera released, NATS closed) |

The page shows a live camera feed plus battery, distance, grayscale, line, cliff, and system
telemetry, all updating with no refresh. The **stop distance (cm)** field sets the proximity
cut-off.

### Safety behavior (enforced in the `picarx` component, not the UI)

- **Watchdog:** if control input stops (release, tab hidden, socket close, Wi-Fi drop), the
  car stops within ~0.5 s. The browser only sends keep-alives while an input is active — the
  watchdog is the real safety mechanism.
- **E-stop:** latches on `X` / STOP / spacebar; `drive` is rejected until cleared.
- **Cliff interlock:** front wheels over an edge → motors stop, a `cliff.event` fires, and
  drive is refused until the cliff clears.
- **Clamping:** every command is clamped to the configured servo/throttle limits at the node.

### Operating from other machines on the mesh

With NATS reachable on the LAN (see [Networking](#networking)), from a second machine:

```sh
gorai mesh services          # lists picarx + camera + teleop-ui
gorai mesh schemas           # lists the registered capability schemas
nats sub 'gorai.picarx.>'    # watch commands + telemetry (video frames are NOT in this stream)
```

Command / telemetry subjects (prefix `gorai.picarx.`):

| Subject | Kind | Payload |
|---|---|---|
| `battery.data` | resource | `{ "volts": number }` |
| `distance.data` | resource | `{ "cm": number }` (`-1` = no echo) |
| `grayscale.data` | resource | `{ "adc": [int,int,int] }` |
| `line.data` | resource | `{ "line": [bool,bool,bool] }` |
| `cliff.data` / `cliff.event` | resource / event | `{ "cliff": bool }` |
| `sysinfo.state` | resource | `{ "fw", "hat", "addr" }` |
| `front.data` | resource | JPEG bytes |
| `drive.command` | tool | `{ "throttle": -100..100 }` |
| `steer.command` | tool | `{ "angle": deg }` (+right / −left) |
| `campan.command` / `camtilt.command` | tool | `{ "angle": deg }` |
| `estop.command` | tool | `{ "clear": bool }` (omit/false = engage) |

Tool replies: `{ "ok": true }` or `{ "ok": false, "error": "<code>", "msg": "..." }` where
`<code>` ∈ `out_of_range | mcu_unavailable | estop_latched | cliff_blocked`.

### Networking

The embedded NATS server binds and dials the **single** URL in `robot.json` → `nats.url`.

- The committed default `nats://127.0.0.1:4222` is **local-only** (the web UI still works;
  off-box mesh access does not).
- For LAN reach, set `nats.url` to the Pi's **routable** address before running, e.g.
  `nats://raspberrypi.local:4222`. Do **not** use `0.0.0.0` — gorai dials the same URL
  locally and rejects it.

> ⚠️ **Security:** the embedded NATS server does not enforce accounts, so exposing it on a
> LAN exposes an *unauthenticated* bus. For any non-isolated network, run an external
> `nats-server` with NKey/JWT accounts and point the robot at it. See
> [`docs/NATS-AUTH.md`](docs/NATS-AUTH.md).

### Calibration

Servo trims and limits live in [`calibration.json`](calibration.json) (`steer`, `pan`,
`tilt` trim/dir/min/max; per-motor scale/invert). Center each servo, record the raw angles,
and edit this file. See the gopicar `picarctl calibrate` tooling.

---

## Build

Builds use the `gorai` CLI (from the [gorai](https://github.com/emergingrobotics/gorai)
toolchain) via the [`Makefile`](Makefile). Run `make` with no target for the full list.

```sh
make build           # build a standalone binary for the host arch  → bin/picarx
make build-arm64     # cross-compile for the Raspberry Pi (linux/arm64)
make run             # build, then run locally with robot.json
make validate        # validate robot.json
make test            # fast, hardware-free tests
make test-hw         # on-Pi hardware integration tests (-tags hardware)
make check           # fmt + vet + test
make install         # copy the binary to ~/bin (override with PREFIX=)
make clean           # remove bin/
```

The build is driven by [`robot.json`](robot.json) (Robot Definition v2), which declares the
`picarx` + `camera` components and the `teleop-ui` service. [`main.go`](main.go) blank-imports
those packages so each self-registers at init.

### Deploy

Cross-compile for arm64 and copy the binary + config to the Pi:

```sh
make deploy DEPLOY_HOST=pi@raspberrypi
```

This runs `build-arm64` and `scp`s `bin/picarx`, `robot.json`, and `calibration.json` to the
target's home directory. Then SSH in and `./picarx run robot.json`.

Full first-boot instructions — camera tags, RTSP status, calibration, and the success-criteria
checklist — are in [`docs/BRINGUP.md`](docs/BRINGUP.md).

---

## Layout

```
main.go                    # blank-import manifest; calls gorai.Run()
robot.json                 # Robot Definition (components + service + NATS config)
calibration.json           # servo trims and limits
Makefile                   # build / run / test / deploy
components/picarx/          # PiCar-X component: resources, tools, safety (clamp/watchdog/e-stop/cliff)
components/camera/          # single Pi-camera capture, fanned out to NATS + MJPEG
services/teleopui/          # embedded web control page + NATS↔browser bridges
  web/                      # vanilla-JS UI (index.html, app.js, style.css)
docs/                       # DESIGN.md, BRINGUP.md, NATS-AUTH.md
VISION.md, REQUIREMENTS.md  # spec (authoritative; design cites requirement IDs)
```

## Docs

- [`docs/DESIGN.md`](docs/DESIGN.md) — architecture, NATS subject map, safety state machine.
- [`docs/BRINGUP.md`](docs/BRINGUP.md) — step-by-step Pi bring-up and acceptance checklist.
- [`docs/NATS-AUTH.md`](docs/NATS-AUTH.md) — securing the LAN-exposed bus.
- [`VISION.md`](VISION.md) / [`REQUIREMENTS.md`](REQUIREMENTS.md) — why this robot exists and
  what it must do.
</content>
</invoke>
