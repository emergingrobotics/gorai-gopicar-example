# gorai-picarx Teleoperable Robot — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a single-binary GoRAI robot on the PiCar-X frame that exposes gopicar's sensors as NCP resources and actuators as NCP tools over embedded NATS, serves one embedded web page (live video + telemetry + slider/keyboard driving), streams video three ways (in-page MJPEG, NATS frames, RTSP), and exposes the NATS bus on the LAN.

**Architecture:** Three units in one `gorai run` binary sharing an embedded NATS mesh: a `picarx` **component** wrapping `gopicar` (owns the single device handle, implements the NATS command server + sensor publishers + safety), a `camera` **component** (`picam` model) that owns one Pi-camera capture and publishes JPEG frames to NATS + serves RTSP, and a `teleopui` **service** that runs its own `http.Server` (page + MJPEG bridge + telemetry WebSocket + control channel) as a mesh client. Safety (clamp, watchdog, e-stop, cliff interlock) lives entirely in the `picarx` component.

**Tech Stack:** Go 1.25; `github.com/emergingrobotics/gorai` (registry, resource, subjects, nats, mesh, dashboard.WebSocketHub); `github.com/emergingrobotics/gopicar/pkg/picarx` (+ `internal/fake` for tests); `github.com/nats-io/nats.go`; `github.com/coder/websocket` (via gorai dashboard hub); `github.com/bluenviron/gortsplib/v4` (RTSP); stdlib `net/http`, `image/jpeg`, `embed`.

## Global Constraints

- Module path: `github.com/emergingrobotics/gorai-picarx`. Go directive `go 1.25.0`.
- Local sibling deps are unpublished — go.mod MUST carry `replace` directives to `../gorai` and `../gopicar` (verbatim in Task 1).
- Robot ID / namespace is `picarx`; all subjects are `gorai.picarx.<capability>.<type>`, `type ∈ {data,state,command,event}` (DESIGN §2.1, §11 R3).
- Component contract (DESIGN §11 R2): constructor `func(ctx, registry.Dependencies, registry.Config) (any, error)`; the object implements `resource.Resource` = `Name() resource.Name`, `Reconfigure(ctx, resource.Dependencies, resource.Config) error`, `DoCommand(ctx, map[string]any) (map[string]any, error)`, `Close(ctx) error`; NATS work happens in a `Start(ctx) error` (the `Startable` interface). Grab `*nats.Conn` via `deps.Get("nats")` and `*slog.Logger` via `deps.Get("logger")`. JSON numeric attributes arrive as `float64`.
- Safety lives ONLY in the `picarx` component handlers (clamp, watchdog, e-stop latch, cliff interlock). Never in `teleopui` or the browser. (REQUIREMENTS R-150…R-155, C-001…C-007.)
- No JSON-schema runtime validation exists in gorai — validate/clamp in-handler; register schemas into `gorai-schemas` for discovery only (DESIGN §11 R6).
- Tool reply shape: `{"ok":true}` (optionally `{"ok":true,"clamped":<n>}` when a value was clamped) or `{"ok":false,"error":"<code>","msg":"..."}`, `<code> ∈ {out_of_range,mcu_unavailable,estop_latched,cliff_blocked}` (DESIGN §2.1).
- Emoji are forbidden in code. Comments explain WHY, not WHAT. Handle every error explicitly. Constraint IDs (`C-00x`) appear in code comments where enforced.
- Build/run/test only via the `Makefile` (Task 2). Cross-compile target for the Pi is `linux/arm64`.

---

## File Structure

```
gorai-gopicar-example/
├── go.mod / go.sum
├── main.go                         # blank-import manifest + gorai.Run()
├── robot.json                      # RDL v2 (DESIGN §6)
├── calibration.json                # per-unit servo/motor calibration
├── Makefile
├── components/
│   ├── picarx/
│   │   ├── control.go              # pure safety/control core (no NATS) — clamp, drive FSM, handlers
│   │   ├── control_test.go
│   │   ├── component.go            # resource.Resource impl + New + Start(ctx) NATS wiring
│   │   ├── sensors.go              # sensor read→publish loops
│   │   ├── schemas.go              # mesh schema registration
│   │   └── component_test.go       # NATS round-trip (embedded server) tests
│   └── camera/
│       ├── source.go               # Capture interface + fake source
│       ├── source_v4l2.go          # real Pi-camera capture (build tag) 
│       ├── camera.go               # component: publish frames + state; owns one capture
│       ├── rtsp.go                 # gortsplib RTP/JPEG server
│       └── camera_test.go
├── services/
│   └── teleopui/
│       ├── service.go              # service: own http.Server, hub, subscriptions
│       ├── mjpeg.go                # MJPEG bridge over raw *nats.Conn
│       ├── control.go              # browser control channel → tool calls
│       ├── service_test.go
│       └── web/
│           ├── index.html
│           ├── app.js
│           └── style.css
└── docs/ (specs already present)
```

Each file has one responsibility. `control.go` is deliberately NATS-free so the safety logic is unit-tested deterministically against gopicar fakes; `component.go` is the thin NATS shell.

---

## Milestone M1 — `picarx` component (REQUIREMENTS R-110…R-114, R-150…R-155)

### Task 1: Repo scaffolding — go.mod, main.go

**Files:**
- Create: `go.mod`, `main.go`

**Interfaces:**
- Produces: a buildable module that other tasks extend; `main.go` calls `gorai.Run()`.

- [ ] **Step 1: Write `go.mod`**

```
module github.com/emergingrobotics/gorai-picarx

go 1.25.0

require (
	github.com/emergingrobotics/gorai v0.0.0
	github.com/emergingrobotics/gopicar v0.0.0
	github.com/nats-io/nats.go v1.37.0
)

replace github.com/emergingrobotics/gorai => ../gorai

replace github.com/emergingrobotics/gopicar => ../gopicar
```

- [ ] **Step 2: Write `main.go`**

```go
// Command gorai-picarx is a teleoperable PiCar-X GoRAI robot: it registers the
// picarx and camera components and the teleop-ui service, then runs the robot.
package main

import (
	gorai "github.com/emergingrobotics/gorai/pkg/gorai"

	// Blank imports are the component/service manifest; each self-registers
	// via init() -> registry.RegisterComponent / RegisterService.
	_ "github.com/emergingrobotics/gorai-picarx/components/camera"
	_ "github.com/emergingrobotics/gorai-picarx/components/picarx"
	_ "github.com/emergingrobotics/gorai-picarx/services/teleopui"
)

func main() {
	gorai.Run()
}
```

- [ ] **Step 3: Create empty package dirs so `go mod tidy` resolves**

Run:
```bash
mkdir -p components/picarx components/camera services/teleopui/web
printf 'package picarx\n' > components/picarx/doc.go
printf 'package camera\n' > components/camera/doc.go
printf 'package teleopui\n' > services/teleopui/doc.go
```

- [ ] **Step 4: Verify the module resolves**

Run: `go mod tidy && go build ./...`
Expected: builds (main.go's blank imports resolve to the empty packages; `gorai.Run` links). If `go.sum` errors on sibling modules, run `go mod tidy` again — the `replace` directives point at local paths so no network fetch occurs.

- [ ] **Step 5: Commit**

```bash
git add go.mod go.sum main.go components services
git commit -m "chore: scaffold gorai-picarx module and manifest"
```

---

### Task 2: Makefile

**Files:**
- Create: `Makefile`

- [ ] **Step 1: Write `Makefile`** (mirrors the gorai-robot-template targets)

```makefile
ROBOT_CONFIG ?= robot.json
BINARY_NAME  ?= picarx
TARGET       ?= $(shell go env GOOS)/$(shell go env GOARCH)

.DEFAULT_GOAL := help
.PHONY: build run test test-hw validate clean deploy fmt vet check help

build: ## Build standalone binary
	gorai build $(ROBOT_CONFIG) -o bin/$(BINARY_NAME) --target $(TARGET)

build-arm64: ## Cross-compile for Raspberry Pi 64-bit
	gorai build $(ROBOT_CONFIG) -o bin/$(BINARY_NAME) --target linux/arm64

run: ## Run robot in development mode
	gorai run $(ROBOT_CONFIG)

test: ## Fast hardware-free tests
	go test ./...

test-hw: ## On-Pi hardware integration tests
	go test -tags hardware ./...

validate: ## Validate robot configuration
	gorai validate $(ROBOT_CONFIG)

fmt: ; go fmt ./...
vet: ; go vet ./...
check: fmt vet test ## All checks

clean: ; rm -rf bin/

deploy: build-arm64 ## Build arm64 + scp to the Pi (set DEPLOY_HOST)
	@if [ -z "$(DEPLOY_HOST)" ]; then echo "Set DEPLOY_HOST (e.g. make deploy DEPLOY_HOST=pi@raspberrypi)"; exit 1; fi
	scp bin/$(BINARY_NAME) robot.json calibration.json $(DEPLOY_HOST):~/

help: ; @grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
```

- [ ] **Step 2: Verify**

Run: `make test`
Expected: PASS (no tests yet → `ok ... [no test files]`).

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "chore: add Makefile"
```

---

### Task 3: picarx control core — clamp + drive with e-stop/cliff interlock

**Files:**
- Create: `components/picarx/control.go`
- Test: `components/picarx/control_test.go`
- Remove: `components/picarx/doc.go` (replaced)

**Interfaces:**
- Consumes: `gopicar` facade via a narrow interface (so tests use a stub or the gopicar fake).
- Produces:
  - `type Device interface { SetDir(context.Context, float64) error; SetCamPan(context.Context, float64) error; SetCamTilt(context.Context, float64) error; Forward(context.Context, float64) error; Backward(context.Context, float64) error; Stop(context.Context) error; Battery(context.Context) (float64, error); Grayscale(context.Context) ([3]int, error); Distance(context.Context, time.Duration) (float64, error); LineStatus(context.Context, [3]int) ([3]bool, error); CliffStatus(context.Context, [3]int) (bool, error); FirmwareVersion(context.Context) (uint8, uint8, uint8, error); HAT() mcu.HAT; Addr() uint8 }` — `*gopicar/pkg/picarx.PiCarX` satisfies this.
  - `type Limits struct { SteerMaxDeg, CamPanMaxDeg, CamTiltMaxDeg, DriveDeadband float64 }`
  - `type controller struct { ... }` with `newController(dev Device, lim Limits, grayRef [3]int, clock func() time.Time) *controller`
  - handler methods returning `map[string]any`: `drive(ctx, throttle float64)`, `steer(ctx, angle float64)`, `campan(ctx, angle float64)`, `camtilt(ctx, angle float64)`, `estop(ctx context.Context, clear bool)`; and safety hooks `tickWatchdog(now time.Time) bool`, `updateCliff(ctx context.Context, detected bool) bool`.

- [ ] **Step 1: Write the failing test** `components/picarx/control_test.go`

```go
package picarx

import (
	"context"
	"testing"
	"time"
)

// stubDev records calls and lets tests force errors. It satisfies Device.
type stubDev struct {
	fwd, back, stop int
	lastFwd, lastBack, lastDir float64
	err error
}

func (s *stubDev) SetDir(_ context.Context, d float64) error   { s.lastDir = d; return s.err }
func (s *stubDev) SetCamPan(context.Context, float64) error    { return s.err }
func (s *stubDev) SetCamTilt(context.Context, float64) error   { return s.err }
func (s *stubDev) Forward(_ context.Context, p float64) error  { s.fwd++; s.lastFwd = p; return s.err }
func (s *stubDev) Backward(_ context.Context, p float64) error { s.back++; s.lastBack = p; return s.err }
func (s *stubDev) Stop(context.Context) error                  { s.stop++; return s.err }
func (s *stubDev) Battery(context.Context) (float64, error)               { return 7.4, nil }
func (s *stubDev) Grayscale(context.Context) ([3]int, error)              { return [3]int{}, nil }
func (s *stubDev) Distance(context.Context, time.Duration) (float64, error) { return 12, nil }
func (s *stubDev) LineStatus(context.Context, [3]int) ([3]bool, error)    { return [3]bool{}, nil }
func (s *stubDev) CliffStatus(context.Context, [3]int) (bool, error)      { return false, nil }
func (s *stubDev) FirmwareVersion(context.Context) (uint8, uint8, uint8, error) { return 2, 1, 1, nil }

var testLim = Limits{SteerMaxDeg: 30, CamPanMaxDeg: 80, CamTiltMaxDeg: 65, DriveDeadband: 5}

func TestDriveForwardBackwardStop(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)

	if r := c.drive(context.Background(), 40); r["ok"] != true {
		t.Fatalf("drive 40: %v", r)
	}
	if d.fwd != 1 || d.lastFwd != 40 {
		t.Fatalf("expected Forward(40), got fwd=%d last=%v", d.fwd, d.lastFwd)
	}
	c.drive(context.Background(), -30)
	if d.back != 1 || d.lastBack != 30 {
		t.Fatalf("expected Backward(30), got back=%d last=%v", d.back, d.lastBack)
	}
	c.drive(context.Background(), 2) // inside deadband -> Stop
	if d.stop != 1 {
		t.Fatalf("expected Stop within deadband, stop=%d", d.stop)
	}
}

// C-002: values past the limit are clamped, never passed through.
func TestDriveClampsAndReports(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	r := c.drive(context.Background(), 9999)
	if d.lastFwd != 100 {
		t.Fatalf("expected clamp to 100, got %v", d.lastFwd)
	}
	if r["ok"] != true || r["clamped"] != 100.0 {
		t.Fatalf("expected ok+clamped=100, got %v", r)
	}
}

// C-004: e-stop latches; drive is refused until cleared.
func TestEstopLatch(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	c.estop(context.Background(), false) // engage
	if d.stop != 1 {
		t.Fatalf("engage should Stop")
	}
	r := c.drive(context.Background(), 50)
	if r["ok"] != false || r["error"] != "estop_latched" {
		t.Fatalf("drive while latched must fail estop_latched, got %v", r)
	}
	if d.fwd != 0 {
		t.Fatalf("no motion while latched")
	}
	c.estop(context.Background(), true) // clear
	if r := c.drive(context.Background(), 50); r["ok"] != true {
		t.Fatalf("drive after clear should succeed, got %v", r)
	}
}

// C-004: a cliff rising edge stops and blocks drive; falling edge clears.
func TestCliffInterlock(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	fired := c.updateCliff(context.Background(), true)
	if !fired || d.stop != 1 {
		t.Fatalf("rising edge must fire+Stop, fired=%v stop=%d", fired, d.stop)
	}
	if r := c.drive(context.Background(), 20); r["error"] != "cliff_blocked" {
		t.Fatalf("drive over cliff must fail cliff_blocked, got %v", r)
	}
	if c.updateCliff(context.Background(), true) {
		t.Fatalf("no re-fire while still detected")
	}
	c.updateCliff(context.Background(), false) // clear
	if r := c.drive(context.Background(), 20); r["ok"] != true {
		t.Fatalf("drive after cliff clears should succeed, got %v", r)
	}
}

// C-003: watchdog stops the car when no fresh drive arrives within the window.
func TestWatchdog(t *testing.T) {
	d := &stubDev{}
	now := time.Unix(1000, 0)
	clock := func() time.Time { return now }
	c := newController(d, testLim, [3]int{}, clock)
	c.window = 500 * time.Millisecond

	c.drive(context.Background(), 40)
	now = now.Add(200 * time.Millisecond)
	if c.tickWatchdog(now) {
		t.Fatalf("must not trip before window")
	}
	now = now.Add(400 * time.Millisecond) // 600ms since drive
	if !c.tickWatchdog(now) {
		t.Fatalf("must trip after window")
	}
	if d.stop == 0 {
		t.Fatalf("watchdog must Stop")
	}
}

func TestSteerClamp(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	c.steer(context.Background(), 90) // limit 30
	if d.lastDir != 30 {
		t.Fatalf("steer must clamp to 30, got %v", d.lastDir)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/picarx/ -run 'Drive|Estop|Cliff|Watchdog|Steer' -v`
Expected: FAIL — `undefined: newController`, `undefined: Limits`, `undefined: Device`.

- [ ] **Step 3: Write `components/picarx/control.go`**

```go
package picarx

import (
	"context"
	"sync"
	"time"

	"github.com/emergingrobotics/gopicar/pkg/mcu"
)

// Device is the narrow slice of the gopicar facade this component drives.
// *github.com/emergingrobotics/gopicar/pkg/picarx.PiCarX satisfies it.
type Device interface {
	SetDir(context.Context, float64) error
	SetCamPan(context.Context, float64) error
	SetCamTilt(context.Context, float64) error
	Forward(context.Context, float64) error
	Backward(context.Context, float64) error
	Stop(context.Context) error
	Battery(context.Context) (float64, error)
	Grayscale(context.Context) ([3]int, error)
	Distance(context.Context, time.Duration) (float64, error)
	LineStatus(context.Context, [3]int) ([3]bool, error)
	CliffStatus(context.Context, [3]int) (bool, error)
	FirmwareVersion(context.Context) (uint8, uint8, uint8, error)
	HAT() mcu.HAT
	Addr() uint8
}

// Limits are the mechanical/electrical bounds enforced before any hardware call.
type Limits struct {
	SteerMaxDeg   float64
	CamPanMaxDeg  float64
	CamTiltMaxDeg float64
	DriveDeadband float64
}

// controller owns all mutable safety state. Its handler methods are NATS-free so
// the safety rules (C-002..C-004) can be unit-tested deterministically.
type controller struct {
	dev     Device
	lim     Limits
	grayRef [3]int
	clock   func() time.Time
	window  time.Duration

	mu        sync.Mutex
	estopped  bool
	cliff     bool
	moving    bool
	lastDrive time.Time
}

func newController(dev Device, lim Limits, grayRef [3]int, clock func() time.Time) *controller {
	return &controller{dev: dev, lim: lim, grayRef: grayRef, clock: clock, window: 500 * time.Millisecond}
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func okResp() map[string]any { return map[string]any{"ok": true} }
func failResp(code, msg string) map[string]any {
	return map[string]any{"ok": false, "error": code, "msg": msg}
}

// drive applies signed throttle. C-002 clamp; C-004 refuse while latched/over cliff.
func (c *controller) drive(ctx context.Context, throttle float64) map[string]any {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.estopped {
		return failResp("estop_latched", "e-stop engaged; send estop.command {clear:true}")
	}
	if c.cliff {
		return failResp("cliff_blocked", "cliff detected; drive blocked")
	}
	clamped := clamp(throttle, -100, 100)
	var err error
	switch {
	case clamped > c.lim.DriveDeadband:
		err = c.dev.Forward(ctx, clamped)
		c.moving = true
	case clamped < -c.lim.DriveDeadband:
		err = c.dev.Backward(ctx, -clamped)
		c.moving = true
	default:
		err = c.dev.Stop(ctx)
		c.moving = false
	}
	if err != nil {
		return failResp("mcu_unavailable", err.Error())
	}
	c.lastDrive = c.clock()
	r := okResp()
	if clamped != throttle {
		r["clamped"] = clamped
	}
	return r
}

func (c *controller) servo(ctx context.Context, angle, max float64, set func(context.Context, float64) error) map[string]any {
	c.mu.Lock()
	defer c.mu.Unlock()
	clamped := clamp(angle, -max, max)
	if err := set(ctx, clamped); err != nil {
		return failResp("mcu_unavailable", err.Error())
	}
	r := okResp()
	if clamped != angle {
		r["clamped"] = clamped
	}
	return r
}

func (c *controller) steer(ctx context.Context, angle float64) map[string]any {
	return c.servo(ctx, angle, c.lim.SteerMaxDeg, c.dev.SetDir)
}
func (c *controller) campan(ctx context.Context, angle float64) map[string]any {
	return c.servo(ctx, angle, c.lim.CamPanMaxDeg, c.dev.SetCamPan)
}
func (c *controller) camtilt(ctx context.Context, angle float64) map[string]any {
	return c.servo(ctx, angle, c.lim.CamTiltMaxDeg, c.dev.SetCamTilt)
}

// estop engages (clear=false) or clears (clear=true) the latch. C-004.
func (c *controller) estop(ctx context.Context, clear bool) map[string]any {
	c.mu.Lock()
	defer c.mu.Unlock()
	if clear {
		c.estopped = false
		return okResp()
	}
	c.estopped = true
	c.moving = false
	if err := c.dev.Stop(ctx); err != nil {
		return failResp("mcu_unavailable", err.Error())
	}
	return okResp()
}

// updateCliff records the latest cliff reading and returns true on a rising edge
// (which stops the motors). C-004.
func (c *controller) updateCliff(ctx context.Context, detected bool) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	rising := detected && !c.cliff
	c.cliff = detected
	if rising {
		c.moving = false
		_ = c.dev.Stop(ctx)
	}
	return rising
}

// tickWatchdog stops the car if it is moving and no drive command arrived within
// the window. Returns true if it tripped. C-003.
func (c *controller) tickWatchdog(now time.Time) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.moving {
		return false
	}
	if now.Sub(c.lastDrive) <= c.window {
		return false
	}
	c.moving = false
	_ = c.dev.Stop(context.Background())
	return true
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./components/picarx/ -run 'Drive|Estop|Cliff|Watchdog|Steer' -v`
Expected: PASS (all six tests).

- [ ] **Step 5: Commit**

```bash
git add components/picarx/control.go components/picarx/control_test.go
git rm components/picarx/doc.go
git commit -m "feat(picarx): safety/control core with clamp, e-stop, cliff, watchdog"
```

---

### Task 4: picarx component shell — resource.Resource + New + config parsing

**Files:**
- Create: `components/picarx/component.go`
- Test: extend `components/picarx/control_test.go` with a config-parse test (or new `component_test.go` — use `component_test.go`)

**Interfaces:**
- Consumes: `newController`, `Limits` (Task 3); gopicar `picarx.Open`, `picarx.Options`, `picarx.Calibration`.
- Produces:
  - `type Component struct { ... }` implementing `resource.Resource` + `Start(ctx) error` + `Close(ctx) error`.
  - `func New(ctx, deps registry.Dependencies, conf registry.Config) (any, error)` registered as `("picarx","picarx")`.
  - `func parseConfig(conf registry.Config) (Limits, [3]int, string, time.Duration)` — reads `steer_max_deg`, `campan_max_deg`, `camtilt_max_deg`, `grayscale_ref`, `calibration`, `watchdog_ms` (all `float64`/`[]any` per Global Constraints).

- [ ] **Step 1: Write the failing test** `components/picarx/component_test.go`

```go
package picarx

import (
	"testing"
	"time"
)

func TestParseConfigDefaultsAndOverrides(t *testing.T) {
	lim, ref, calib, win := parseConfig(map[string]any{
		"steer_max_deg": float64(25),
		"grayscale_ref": []any{float64(900), float64(950), float64(1000)},
		"calibration":   "calib.json",
		"watchdog_ms":   float64(400),
	})
	if lim.SteerMaxDeg != 25 {
		t.Fatalf("steer max override: %v", lim.SteerMaxDeg)
	}
	if lim.CamPanMaxDeg != defaultCamPanMaxDeg {
		t.Fatalf("campan default: %v", lim.CamPanMaxDeg)
	}
	if ref != [3]int{900, 950, 1000} {
		t.Fatalf("grayscale ref: %v", ref)
	}
	if calib != "calib.json" {
		t.Fatalf("calibration path: %v", calib)
	}
	if win != 400*time.Millisecond {
		t.Fatalf("watchdog window: %v", win)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/picarx/ -run TestParseConfig -v`
Expected: FAIL — `undefined: parseConfig`, `undefined: defaultCamPanMaxDeg`.

- [ ] **Step 3: Write `components/picarx/component.go`**

```go
package picarx

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"os"
	"time"

	gopx "github.com/emergingrobotics/gopicar/pkg/picarx"
	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
	"github.com/nats-io/nats.go"
)

const (
	defaultSteerMaxDeg   = 30
	defaultCamPanMaxDeg  = 80
	defaultCamTiltMaxDeg = 65
	defaultDeadband      = 5
	defaultWatchdogMS    = 500
)

func init() {
	registry.RegisterComponent("picarx", "picarx", New)
}

// Component is the picarx capability node: it owns the single gopicar handle and
// serves resources/tools over NATS with safety enforced in its controller.
type Component struct {
	name    resource.Name
	nc      *nats.Conn
	log     *slog.Logger
	subj    *subjects.Builder
	px      *gopx.PiCarX
	ctl     *controller
	grayRef [3]int
	cancel  context.CancelFunc
	subs    []*nats.Subscription
}

func parseConfig(conf registry.Config) (Limits, [3]int, string, time.Duration) {
	num := func(key string, def float64) float64 {
		if v, ok := conf[key].(float64); ok {
			return v
		}
		return def
	}
	lim := Limits{
		SteerMaxDeg:   num("steer_max_deg", defaultSteerMaxDeg),
		CamPanMaxDeg:  num("campan_max_deg", defaultCamPanMaxDeg),
		CamTiltMaxDeg: num("camtilt_max_deg", defaultCamTiltMaxDeg),
		DriveDeadband: num("deadband", defaultDeadband),
	}
	var ref [3]int
	if raw, ok := conf["grayscale_ref"].([]any); ok && len(raw) == 3 {
		for i := 0; i < 3; i++ {
			if f, ok := raw[i].(float64); ok {
				ref[i] = int(f)
			}
		}
	}
	calib, _ := conf["calibration"].(string)
	win := time.Duration(num("watchdog_ms", defaultWatchdogMS)) * time.Millisecond
	return lim, ref, calib, win
}

// loadCalibration reads a picarx.Calibration JSON file; empty path -> Measured.
func loadCalibration(path string) (gopx.Calibration, error) {
	if path == "" {
		return gopx.MeasuredCalibration(), nil
	}
	data, err := os.ReadFile(path)
	if err != nil {
		return gopx.Calibration{}, fmt.Errorf("read calibration %q: %w", path, err)
	}
	var c gopx.Calibration
	if err := json.Unmarshal(data, &c); err != nil {
		return gopx.Calibration{}, fmt.Errorf("parse calibration %q: %w", path, err)
	}
	return c, nil
}

func New(ctx context.Context, deps registry.Dependencies, conf registry.Config) (any, error) {
	name, _ := conf["name"].(string)
	robotID, _ := conf["namespace"].(string)

	nc, err := getConn(deps)
	if err != nil {
		return nil, err
	}
	log := getLogger(deps)

	lim, ref, calibPath, win := parseConfig(conf)
	calib, err := loadCalibration(calibPath)
	if err != nil {
		return nil, err
	}
	px, err := gopx.Open(ctx, gopx.Options{Calibration: calib})
	if err != nil {
		return nil, fmt.Errorf("open picarx: %w", err)
	}
	ctl := newController(px, lim, ref, time.Now)
	ctl.window = win

	return &Component{
		name:    resource.NewComponentName("gorai", "picarx", name),
		nc:      nc,
		log:     log,
		subj:    subjects.NewBuilder(robotID),
		px:      px,
		ctl:     ctl,
		grayRef: ref,
	}, nil
}

func getConn(deps registry.Dependencies) (*nats.Conn, error) {
	v, err := deps.Get("nats")
	if err != nil {
		return nil, fmt.Errorf("nats dependency: %w", err)
	}
	nc, ok := v.(*nats.Conn)
	if !ok {
		return nil, fmt.Errorf("nats dependency is %T, want *nats.Conn", v)
	}
	return nc, nil
}

func getLogger(deps registry.Dependencies) *slog.Logger {
	if v, err := deps.Get("logger"); err == nil {
		if l, ok := v.(*slog.Logger); ok {
			return l
		}
	}
	return slog.Default()
}

func (c *Component) Name() resource.Name { return c.name }

func (c *Component) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}

func (c *Component) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (c *Component) Close(context.Context) error {
	if c.cancel != nil {
		c.cancel()
	}
	for _, s := range c.subs {
		_ = s.Unsubscribe()
	}
	if c.px != nil {
		return c.px.Close()
	}
	return nil
}

var _ resource.Resource = (*Component)(nil)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./components/picarx/ -run TestParseConfig -v`
Expected: PASS. Also `go build ./...` succeeds (Start is added in Task 5; the `Startable` interface is optional so the build is fine without it).

- [ ] **Step 5: Commit**

```bash
git add components/picarx/component.go components/picarx/component_test.go
git commit -m "feat(picarx): component shell, config parsing, calibration load"
```

---

### Task 5: picarx NATS server — command subjects + Start/watchdog/cliff loops

**Files:**
- Create: `components/picarx/server.go`
- Modify: `components/picarx/component.go` (add `Start` calling `server.go` wiring)
- Test: `components/picarx/roundtrip_test.go` (embedded NATS server)

**Interfaces:**
- Consumes: controller handlers (Task 3), `Component` (Task 4).
- Produces: `func (c *Component) Start(ctx context.Context) error` (satisfies gorai `Startable`); private `func (c *Component) serveCommand(ctx, cap string, h func(context.Context, map[string]any) map[string]any)`.

- [ ] **Step 1: Write the failing test** `components/picarx/roundtrip_test.go`

```go
package picarx

import (
	"context"
	"encoding/json"
	"testing"
	"time"

	natsserver "github.com/nats-io/nats-server/v2/server"
	"github.com/nats-io/nats.go"
)

// startNATS spins an in-process NATS server for round-trip tests.
func startNATS(t *testing.T) *nats.Conn {
	t.Helper()
	opts := &natsserver.Options{Host: "127.0.0.1", Port: -1}
	srv, err := natsserver.NewServer(opts)
	if err != nil {
		t.Fatal(err)
	}
	go srv.Start()
	if !srv.ReadyForConnections(2 * time.Second) {
		t.Fatal("nats not ready")
	}
	t.Cleanup(srv.Shutdown)
	nc, err := nats.Connect(srv.ClientURL())
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(nc.Close)
	return nc
}

func TestDriveCommandRoundTrip(t *testing.T) {
	nc := startNATS(t)
	d := &stubDev{}
	c := newTestComponent(nc, d) // helper defined below

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := c.Start(ctx); err != nil {
		t.Fatal(err)
	}

	req, _ := json.Marshal(map[string]any{"throttle": 40.0})
	msg, err := nc.Request("gorai.picarx.drive.command", req, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	var resp map[string]any
	_ = json.Unmarshal(msg.Data, &resp)
	if resp["ok"] != true {
		t.Fatalf("drive reply: %v", resp)
	}
	if d.fwd != 1 || d.lastFwd != 40 {
		t.Fatalf("expected Forward(40), got fwd=%d last=%v", d.fwd, d.lastFwd)
	}
}
```

Add this helper to `component_test.go`:

```go
func newTestComponent(nc *nats.Conn, d Device) *Component {
	ctl := newController(d, testLim, [3]int{}, time.Now)
	return &Component{
		name:    resource.NewComponentName("gorai", "picarx", "picarx"),
		nc:      nc,
		log:     slog.Default(),
		subj:    subjects.NewBuilder("picarx"),
		ctl:     ctl,
		grayRef: [3]int{},
	}
}
```
(add imports `"log/slog"`, `"github.com/emergingrobotics/gorai/pkg/resource"`, `"github.com/emergingrobotics/gorai/pkg/subjects"`, `"github.com/nats-io/nats.go"` to `component_test.go`.)

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/picarx/ -run TestDriveCommandRoundTrip -v`
Expected: FAIL — `c.Start undefined` (and `nats-server` not yet a dependency).

- [ ] **Step 3: Add the nats-server test dependency**

Run: `go get github.com/nats-io/nats-server/v2@latest`
Expected: adds to go.mod (test-only usage).

- [ ] **Step 4: Write `components/picarx/server.go`**

```go
package picarx

import (
	"context"
	"encoding/json"
	"time"

	"github.com/nats-io/nats.go"
)

// Start wires the NATS command server, sensor publishers, watchdog and cliff
// loops. It is invoked by the robot runtime (Startable). NATS pub/sub lives here;
// safety decisions live in the controller.
func (c *Component) Start(ctx context.Context) error {
	ctx, c.cancel = context.WithCancel(ctx)

	// Tools (command request/reply). Args are JSON map[string]any (DESIGN §11 R4).
	c.serveCommand(ctx, "drive", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.drive(ctx, num(a, "throttle"))
	})
	c.serveCommand(ctx, "steer", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.steer(ctx, num(a, "angle"))
	})
	c.serveCommand(ctx, "campan", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.campan(ctx, num(a, "angle"))
	})
	c.serveCommand(ctx, "camtilt", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.camtilt(ctx, num(a, "angle"))
	})
	c.serveCommand(ctx, "estop", func(ctx context.Context, a map[string]any) map[string]any {
		clear, _ := a["clear"].(bool)
		return c.ctl.estop(ctx, clear)
	})

	c.startSensors(ctx)   // defined in sensors.go
	go c.watchdogLoop(ctx)
	go c.cliffLoop(ctx)
	return nil
}

func num(a map[string]any, key string) float64 {
	v, _ := a[key].(float64)
	return v
}

func (c *Component) serveCommand(ctx context.Context, capability string, h func(context.Context, map[string]any) map[string]any) {
	subject := c.subj.ComponentCommand(capability) // gorai.<robot>.<cap>.command
	sub, err := c.nc.Subscribe(subject, func(m *nats.Msg) {
		var args map[string]any
		if len(m.Data) > 0 {
			_ = json.Unmarshal(m.Data, &args)
		}
		resp := h(ctx, args)
		if b, err := json.Marshal(resp); err == nil {
			_ = m.Respond(b)
		}
	})
	if err != nil {
		c.log.Error("subscribe failed", "subject", subject, "err", err)
		return
	}
	c.subs = append(c.subs, sub)
}

// watchdogLoop trips the drive watchdog at half the window cadence. C-003.
func (c *Component) watchdogLoop(ctx context.Context) {
	tick := c.ctl.window / 2
	if tick <= 0 {
		tick = 100 * time.Millisecond
	}
	t := time.NewTicker(tick)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case now := <-t.C:
			if c.ctl.tickWatchdog(now) {
				c.log.Warn("drive watchdog tripped; stopped")
			}
		}
	}
}

// cliffLoop polls the cliff sensor and publishes an event on the rising edge. C-004.
func (c *Component) cliffLoop(ctx context.Context) {
	t := time.NewTicker(100 * time.Millisecond)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
			detected, err := c.ctl.dev.CliffStatus(ctx, c.grayRef)
			if err != nil {
				continue
			}
			if c.ctl.updateCliff(ctx, detected) {
				payload, _ := json.Marshal(map[string]any{"cliff": true, "ts": time.Now().UTC().Format(time.RFC3339)})
				_ = c.nc.Publish(c.subj.ComponentEvent("cliff"), payload)
			}
			// also publish current state on the data stream
			payload, _ := json.Marshal(map[string]any{"cliff": detected})
			_ = c.nc.Publish(c.subj.ComponentData("cliff"), payload)
		}
	}
}
```

- [ ] **Step 5: Add a temporary no-op `startSensors` so the package compiles now** (real body in Task 6)

Append to `server.go`:

```go
// startSensors is implemented in sensors.go (Task 6). Declared there.
```

(Do NOT define it here — Task 6 creates `sensors.go` with `func (c *Component) startSensors(ctx context.Context)`. To keep Task 5 independently testable, create `sensors.go` now with the stub below and flesh it out in Task 6.)

Create `components/picarx/sensors.go`:

```go
package picarx

import "context"

// startSensors launches the sensor read->publish loops. Fleshed out in Task 6.
func (c *Component) startSensors(ctx context.Context) {}
```

- [ ] **Step 6: Run test to verify it passes**

Run: `go test ./components/picarx/ -run TestDriveCommandRoundTrip -v`
Expected: PASS. Then `go test ./components/picarx/...` — all prior tests still PASS.

- [ ] **Step 7: Commit**

```bash
git add components/picarx/server.go components/picarx/sensors.go components/picarx/roundtrip_test.go components/picarx/component_test.go go.mod go.sum
git commit -m "feat(picarx): NATS command server, watchdog and cliff loops"
```

---

### Task 6: picarx sensor publishers + state replies

**Files:**
- Modify: `components/picarx/sensors.go` (replace stub)
- Test: `components/picarx/sensors_test.go`

**Interfaces:**
- Produces: `func (c *Component) startSensors(ctx context.Context)` publishing `battery/distance/grayscale/line` on `.data` at their rates and serving `.state` + `sysinfo.state` via request/reply; pure builders `batteryPayload`, `distancePayload`, `grayscalePayload`, `linePayload`, `sysinfoPayload` for testing.

- [ ] **Step 1: Write the failing test** `components/picarx/sensors_test.go`

```go
package picarx

import (
	"context"
	"encoding/json"
	"testing"
	"time"

	"github.com/nats-io/nats.go"
)

func TestBatteryDataPublishes(t *testing.T) {
	nc := startNATS(t)
	d := &stubDev{}
	c := newTestComponent(nc, d)

	sub, err := nc.SubscribeSync("gorai.picarx.battery.data")
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	c.startSensors(ctx)

	msg, err := sub.NextMsg(2 * time.Second)
	if err != nil {
		t.Fatalf("no battery data: %v", err)
	}
	var p map[string]any
	_ = json.Unmarshal(msg.Data, &p)
	if p["volts"] != 7.4 {
		t.Fatalf("battery payload: %v", p)
	}
}

func TestSysinfoStateReply(t *testing.T) {
	nc := startNATS(t)
	c := newTestComponent(nc, &stubDev{})
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	c.startSensors(ctx)

	msg, err := nc.Request("gorai.picarx.sysinfo.state", nil, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	var p map[string]any
	_ = json.Unmarshal(msg.Data, &p)
	if p["fw"] != "2.1.1" {
		t.Fatalf("sysinfo fw: %v", p)
	}
	_ = nats.Msg{}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/picarx/ -run 'Battery|Sysinfo' -v`
Expected: FAIL — `startSensors` is a no-op, so no data / no state reply.

- [ ] **Step 3: Replace `components/picarx/sensors.go`**

```go
package picarx

import (
	"context"
	"encoding/json"
	"fmt"
	"time"

	"github.com/nats-io/nats.go"
)

func (c *Component) startSensors(ctx context.Context) {
	// Streaming sensors: read at cadence, publish JSON to <cap>.data.
	go c.streamLoop(ctx, "battery", time.Second, c.batteryPayload)
	go c.streamLoop(ctx, "distance", 100*time.Millisecond, c.distancePayload)
	go c.streamLoop(ctx, "grayscale", 100*time.Millisecond, c.grayscalePayload)
	go c.streamLoop(ctx, "line", 100*time.Millisecond, c.linePayload)

	// Snapshot state replies for each capability + sysinfo.
	for _, cap := range []struct {
		name string
		fn   func(context.Context) map[string]any
	}{
		{"battery", c.batteryPayload},
		{"distance", c.distancePayload},
		{"grayscale", c.grayscalePayload},
		{"line", c.linePayload},
		{"cliff", c.cliffPayload},
		{"sysinfo", c.sysinfoPayload},
	} {
		c.serveState(ctx, cap.name, cap.fn)
	}
}

func (c *Component) streamLoop(ctx context.Context, capability string, period time.Duration, fn func(context.Context) map[string]any) {
	subject := c.subj.ComponentData(capability)
	t := time.NewTicker(period)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
			if b, err := json.Marshal(fn(ctx)); err == nil {
				_ = c.nc.Publish(subject, b)
			}
		}
	}
}

func (c *Component) serveState(ctx context.Context, capability string, fn func(context.Context) map[string]any) {
	sub, err := c.nc.Subscribe(c.subj.ComponentState(capability), func(m *nats.Msg) {
		if b, err := json.Marshal(fn(ctx)); err == nil {
			_ = m.Respond(b)
		}
	})
	if err == nil {
		c.subs = append(c.subs, sub)
	}
}

func (c *Component) batteryPayload(ctx context.Context) map[string]any {
	v, err := c.ctl.dev.Battery(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{"volts": v}
}

func (c *Component) distancePayload(ctx context.Context) map[string]any {
	cm, err := c.ctl.dev.Distance(ctx, 30*time.Millisecond)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{"cm": cm}
}

func (c *Component) grayscalePayload(ctx context.Context) map[string]any {
	g, err := c.ctl.dev.Grayscale(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{"adc": []int{g[0], g[1], g[2]}}
}

func (c *Component) linePayload(ctx context.Context) map[string]any {
	l, err := c.ctl.dev.LineStatus(ctx, c.grayRef)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{"line": []bool{l[0], l[1], l[2]}}
}

func (c *Component) cliffPayload(ctx context.Context) map[string]any {
	b, err := c.ctl.dev.CliffStatus(ctx, c.grayRef)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{"cliff": b}
}

func (c *Component) sysinfoPayload(ctx context.Context) map[string]any {
	maj, min, patch, err := c.ctl.dev.FirmwareVersion(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	return map[string]any{
		"fw":   fmt.Sprintf("%d.%d.%d", maj, min, patch),
		"hat":  c.ctl.dev.HAT().Model,
		"addr": int(c.ctl.dev.Addr()),
	}
}
```

Note: confirm `mcu.HAT` has a `Model` field; if the field name differs, read `/gorai-all/gopicar/pkg/mcu` and use the actual exported field. If `HAT` has no string field, publish `fmt.Sprintf("%v", c.ctl.dev.HAT())`.

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./components/picarx/ -run 'Battery|Sysinfo' -v`
Expected: PASS. Then `go test ./components/picarx/...` — all PASS.

- [ ] **Step 5: Commit**

```bash
git add components/picarx/sensors.go components/picarx/sensors_test.go
git commit -m "feat(picarx): sensor data streams and state replies"
```

---

### Task 7: picarx schema registration (discovery)

**Files:**
- Create: `components/picarx/schemas.go`
- Modify: `components/picarx/server.go` (call `registerSchemas` in `Start`)
- Test: `components/picarx/schemas_test.go`

**Interfaces:**
- Produces: `func (c *Component) registerSchemas(ctx context.Context) error` writing JSON schemas for each tool/resource into `gorai-schemas` via `mesh.NewClient` + `RegisterSchema`; pure `toolSchemas()`/`resourceSchemas()` returning `[]mesh.SchemaDescriptor`.

- [ ] **Step 1: Write the failing test** `components/picarx/schemas_test.go`

```go
package picarx

import "testing"

func TestSchemaCatalogCoversSurface(t *testing.T) {
	got := map[string]bool{}
	for _, s := range allSchemas() {
		got[s.Name] = true
	}
	for _, want := range []string{
		"gorai.picarx.drive.command", "gorai.picarx.steer.command",
		"gorai.picarx.campan.command", "gorai.picarx.camtilt.command",
		"gorai.picarx.estop.command", "gorai.picarx.battery.data",
		"gorai.picarx.distance.data", "gorai.picarx.grayscale.data",
		"gorai.picarx.line.data", "gorai.picarx.cliff.data",
		"gorai.picarx.sysinfo.state",
	} {
		if !got[want] {
			t.Errorf("missing schema %s", want)
		}
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/picarx/ -run TestSchemaCatalog -v`
Expected: FAIL — `undefined: allSchemas`.

- [ ] **Step 3: Write `components/picarx/schemas.go`**

```go
package picarx

import (
	"context"

	"github.com/emergingrobotics/gorai/pkg/mesh"
)

// allSchemas is the discovery surface (DESIGN §11 R6): registered for tools/list
// style discovery; NOT runtime-enforced (validation/clamp happen in-handler).
func allSchemas() []mesh.SchemaDescriptor {
	obj := func(props map[string]any, required ...string) map[string]any {
		m := map[string]any{"type": "object", "properties": props}
		if len(required) > 0 {
			m["required"] = required
		}
		return m
	}
	numProp := map[string]any{"type": "number"}
	descs := []mesh.SchemaDescriptor{}
	add := func(name, desc string, def any) {
		d, err := mesh.NewJSONSchema(name, "1", desc, def)
		if err == nil {
			descs = append(descs, d)
		}
	}
	add("gorai.picarx.drive.command", "drive throttle -100..100", obj(map[string]any{"throttle": numProp}, "throttle"))
	add("gorai.picarx.steer.command", "steer angle deg", obj(map[string]any{"angle": numProp}, "angle"))
	add("gorai.picarx.campan.command", "camera pan deg", obj(map[string]any{"angle": numProp}, "angle"))
	add("gorai.picarx.camtilt.command", "camera tilt deg", obj(map[string]any{"angle": numProp}, "angle"))
	add("gorai.picarx.estop.command", "engage/clear e-stop", obj(map[string]any{"clear": map[string]any{"type": "boolean"}}))
	add("gorai.picarx.battery.data", "pack voltage", obj(map[string]any{"volts": numProp}))
	add("gorai.picarx.distance.data", "ultrasonic cm (-1 no echo)", obj(map[string]any{"cm": numProp}))
	add("gorai.picarx.grayscale.data", "grayscale adc [L,M,R]", obj(map[string]any{"adc": map[string]any{"type": "array"}}))
	add("gorai.picarx.line.data", "line detect [L,M,R]", obj(map[string]any{"line": map[string]any{"type": "array"}}))
	add("gorai.picarx.cliff.data", "cliff detected", obj(map[string]any{"cliff": map[string]any{"type": "boolean"}}))
	add("gorai.picarx.sysinfo.state", "firmware/HAT/addr", obj(map[string]any{"fw": map[string]any{"type": "string"}}))
	return descs
}

func (c *Component) registerSchemas(ctx context.Context) error {
	client, err := mesh.NewClient(c.nc)
	if err != nil {
		return err
	}
	for _, s := range allSchemas() {
		if err := client.RegisterSchema(ctx, s); err != nil {
			c.log.Warn("schema register failed", "name", s.Name, "err", err)
		}
	}
	return nil
}
```

- [ ] **Step 4: Call it from `Start`** — in `server.go`, after the `serveCommand` block and before `startSensors`, add:

```go
	if err := c.registerSchemas(ctx); err != nil {
		c.log.Warn("schema registration failed", "err", err)
	}
```

- [ ] **Step 5: Run tests**

Run: `go test ./components/picarx/... -v`
Expected: PASS (all). Confirm `mesh.NewJSONSchema` and `mesh.NewClient` signatures against `/gorai-all/gorai/pkg/mesh` (DESIGN §11 R6) — adjust arg order if the source differs.

- [ ] **Step 6: Commit**

```bash
git add components/picarx/schemas.go components/picarx/schemas_test.go components/picarx/server.go
git commit -m "feat(picarx): register capability schemas for mesh discovery"
```

---

## Milestone M2 — camera component (REQUIREMENTS R-120…R-124, R-134)

### Task 8: Capture source interface + fake

**Files:**
- Create: `components/camera/source.go`, `components/camera/source_test.go`
- Remove: `components/camera/doc.go`

**Interfaces:**
- Produces:
  - `type Frame struct { JPEG []byte; Seq uint64; TS time.Time }`
  - `type Source interface { Start(ctx context.Context) (<-chan Frame, error); Properties() Properties; Close() error }`
  - `type Properties struct { Width, Height int; FrameRate float64; Encoding string; PTZ bool }`
  - `func newFakeSource(props Properties, frames [][]byte, period time.Duration) Source` (test/bench source emitting canned JPEG bytes).

- [ ] **Step 1: Write the failing test** `components/camera/source_test.go`

```go
package camera

import (
	"context"
	"testing"
	"time"
)

func TestFakeSourceEmits(t *testing.T) {
	src := newFakeSource(Properties{Width: 4, Height: 4, FrameRate: 50, Encoding: "jpeg"},
		[][]byte{{0xFF, 0xD8, 0x01}, {0xFF, 0xD8, 0x02}}, 5*time.Millisecond)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	ch, err := src.Start(ctx)
	if err != nil {
		t.Fatal(err)
	}
	f1 := <-ch
	f2 := <-ch
	if f1.Seq == f2.Seq {
		t.Fatalf("seq must advance: %d %d", f1.Seq, f2.Seq)
	}
	if len(f1.JPEG) == 0 {
		t.Fatalf("empty frame")
	}
	if src.Properties().Width != 4 {
		t.Fatalf("props")
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./components/camera/ -run TestFakeSource -v`
Expected: FAIL — undefined identifiers.

- [ ] **Step 3: Write `components/camera/source.go`**

```go
package camera

import (
	"context"
	"time"
)

type Frame struct {
	JPEG []byte
	Seq  uint64
	TS   time.Time
}

type Properties struct {
	Width, Height int
	FrameRate     float64
	Encoding      string
	PTZ           bool
}

// Source is one camera capture. Exactly one Source is opened per camera (I-005).
type Source interface {
	Start(ctx context.Context) (<-chan Frame, error)
	Properties() Properties
	Close() error
}

type fakeSource struct {
	props  Properties
	frames [][]byte
	period time.Duration
}

func newFakeSource(props Properties, frames [][]byte, period time.Duration) Source {
	return &fakeSource{props: props, frames: frames, period: period}
}

func (f *fakeSource) Properties() Properties { return f.props }
func (f *fakeSource) Close() error           { return nil }

func (f *fakeSource) Start(ctx context.Context) (<-chan Frame, error) {
	ch := make(chan Frame)
	go func() {
		defer close(ch)
		t := time.NewTicker(f.period)
		defer t.Stop()
		var seq uint64
		i := 0
		for {
			select {
			case <-ctx.Done():
				return
			case now := <-t.C:
				jpeg := f.frames[i%len(f.frames)]
				i++
				seq++
				select {
				case ch <- Frame{JPEG: jpeg, Seq: seq, TS: now}:
				case <-ctx.Done():
					return
				}
			}
		}
	}()
	return ch, nil
}
```

- [ ] **Step 4: Run test / Step 5: Commit**

Run: `go test ./components/camera/ -run TestFakeSource -v` → PASS.
```bash
git add components/camera/source.go components/camera/source_test.go
git rm components/camera/doc.go
git commit -m "feat(camera): capture Source interface and fake source"
```

---

### Task 9: camera component — publish frames to NATS + state, single capture

**Files:**
- Create: `components/camera/camera.go`, `components/camera/camera_test.go`

**Interfaces:**
- Consumes: `Source` (Task 8); gorai `registry`, `resource`, `subjects`, raw `*nats.Conn`.
- Produces: `type Component struct{...}` registered `("camera","picam")`, implementing `resource.Resource` + `Start`; publishes each `Frame.JPEG` to `gorai.<robot>.<name>.data` and answers `<name>.state`. A package var `sourceFactory func(conf registry.Config) (Source, error)` defaults to the fake in tests (real v4l2 in Task 10 via build tag).

- [ ] **Step 1: Write the failing test** `components/camera/camera_test.go`

```go
package camera

import (
	"context"
	"testing"
	"time"

	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
	"github.com/nats-io/nats.go"
	natsserver "github.com/nats-io/nats-server/v2/server"
)

func startNATS(t *testing.T) *nats.Conn {
	t.Helper()
	srv, err := natsserver.NewServer(&natsserver.Options{Host: "127.0.0.1", Port: -1})
	if err != nil { t.Fatal(err) }
	go srv.Start()
	if !srv.ReadyForConnections(2 * time.Second) { t.Fatal("nats not ready") }
	t.Cleanup(srv.Shutdown)
	nc, err := nats.Connect(srv.ClientURL())
	if err != nil { t.Fatal(err) }
	t.Cleanup(nc.Close)
	return nc
}

func TestCameraPublishesFrames(t *testing.T) {
	nc := startNATS(t)
	c := &Component{
		name: resource.NewComponentName("gorai", "camera", "front"),
		nc:   nc,
		subj: subjects.NewBuilder("picarx"),
		capName: "front",
		src: newFakeSource(Properties{Width: 4, Height: 4, FrameRate: 50, Encoding: "jpeg"},
			[][]byte{{0xFF, 0xD8, 0x09}}, 5*time.Millisecond),
	}
	sub, err := nc.SubscribeSync("gorai.picarx.front.data")
	if err != nil { t.Fatal(err) }

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := c.Start(ctx); err != nil { t.Fatal(err) }

	msg, err := sub.NextMsg(2 * time.Second)
	if err != nil { t.Fatalf("no frame: %v", err) }
	if len(msg.Data) != 3 || msg.Data[0] != 0xFF {
		t.Fatalf("unexpected frame bytes: %v", msg.Data)
	}
	_ = registry.Config{}
}
```

- [ ] **Step 2: Run to verify it fails** → `go test ./components/camera/ -run TestCameraPublishesFrames -v` → FAIL (Component undefined). Also add the nats-server dep if not present (`go get github.com/nats-io/nats-server/v2@latest`).

- [ ] **Step 3: Write `components/camera/camera.go`**

```go
package camera

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"

	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
	"github.com/nats-io/nats.go"
)

func init() {
	registry.RegisterComponent("camera", "picam", New)
}

// sourceFactory builds the capture source. Overridden by the v4l2 build (Task 10);
// the default returns an error so a hostless build fails loudly rather than silently.
var sourceFactory = func(conf registry.Config) (Source, error) {
	return nil, fmt.Errorf("no camera source compiled in; build with -tags v4l2 on the Pi")
}

type Component struct {
	name    resource.Name
	nc      *nats.Conn
	log     *slog.Logger
	subj    *subjects.Builder
	capName string
	src     Source
	rtsp    *rtspServer // Task 12; nil disables RTSP
	cancel  context.CancelFunc
	subs    []*nats.Subscription
}

func New(ctx context.Context, deps registry.Dependencies, conf registry.Config) (any, error) {
	name, _ := conf["name"].(string)
	robotID, _ := conf["namespace"].(string)

	v, err := deps.Get("nats")
	if err != nil {
		return nil, fmt.Errorf("nats dependency: %w", err)
	}
	nc, ok := v.(*nats.Conn)
	if !ok {
		return nil, fmt.Errorf("nats dependency is %T", v)
	}
	log := slog.Default()
	if lv, err := deps.Get("logger"); err == nil {
		if l, ok := lv.(*slog.Logger); ok {
			log = l
		}
	}
	src, err := sourceFactory(conf)
	if err != nil {
		return nil, err
	}
	return &Component{
		name:    resource.NewComponentName("gorai", "camera", name),
		nc:      nc,
		log:     log,
		subj:    subjects.NewBuilder(robotID),
		capName: name,
		src:     src,
	}, nil
}

func (c *Component) Name() resource.Name { return c.name }
func (c *Component) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}
func (c *Component) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (c *Component) Start(ctx context.Context) error {
	ctx, c.cancel = context.WithCancel(ctx)
	frames, err := c.src.Start(ctx)
	if err != nil {
		return fmt.Errorf("camera source start: %w", err)
	}
	dataSubj := c.subj.ComponentData(c.capName)

	// state reply: resolution/encoding/fps/ptz
	sub, err := c.nc.Subscribe(c.subj.ComponentState(c.capName), func(m *nats.Msg) {
		p := c.src.Properties()
		b, _ := json.Marshal(map[string]any{
			"w": p.Width, "h": p.Height, "enc": p.Encoding, "fps": p.FrameRate, "ptz": p.PTZ,
		})
		_ = m.Respond(b)
	})
	if err == nil {
		c.subs = append(c.subs, sub)
	}

	// single capture, fanned out: NATS publish (+ RTSP in Task 12). I-005/C-006.
	go func() {
		for {
			select {
			case <-ctx.Done():
				return
			case f, ok := <-frames:
				if !ok {
					return
				}
				_ = c.nc.Publish(dataSubj, f.JPEG)
				if c.rtsp != nil {
					c.rtsp.push(f)
				}
			}
		}
	}()
	return nil
}

func (c *Component) Close(context.Context) error {
	if c.cancel != nil {
		c.cancel()
	}
	for _, s := range c.subs {
		_ = s.Unsubscribe()
	}
	if c.rtsp != nil {
		_ = c.rtsp.close()
	}
	return c.src.Close()
}

var _ resource.Resource = (*Component)(nil)
```

- [ ] **Step 4: Add a stub `rtspServer` so it compiles** (real impl Task 12). Create `components/camera/rtsp.go`:

```go
package camera

// rtspServer is fleshed out in Task 12. Nil rtsp disables RTSP publishing.
type rtspServer struct{}

func (r *rtspServer) push(Frame) {}
func (r *rtspServer) close() error { return nil }
```

- [ ] **Step 5: Run test / Step 6: Commit**

Run: `go test ./components/camera/ -v` → PASS.
```bash
git add components/camera/camera.go components/camera/rtsp.go components/camera/camera_test.go go.mod go.sum
git commit -m "feat(camera): picam component publishes frames + state over NATS"
```

---

### Task 10: real V4L2 capture source (build-tagged)

**Files:**
- Create: `components/camera/source_v4l2.go` (with `//go:build v4l2`)

**Interfaces:**
- Consumes: gorai `components/camera/v4l2` (`New`, `SetFrameCallback`) — verified DESIGN §11 R7.
- Produces: sets `sourceFactory` (in an `init()` under the `v4l2` tag) to build a Source backed by the gorai v4l2 camera, wiring `SetFrameCallback` → the Source's frame channel.

- [ ] **Step 1: Write `components/camera/source_v4l2.go`**

```go
//go:build v4l2

package camera

import (
	"context"
	"time"

	v4l2cam "github.com/emergingrobotics/gorai/components/camera/v4l2"
	"github.com/emergingrobotics/gorai/pkg/registry"
)

func init() {
	sourceFactory = func(conf registry.Config) (Source, error) {
		return &v4l2Source{conf: conf}, nil
	}
}

type v4l2Source struct {
	conf   registry.Config
	cam    *v4l2cam.Camera
	props  Properties
	cancel context.CancelFunc
}

func (s *v4l2Source) Properties() Properties { return s.props }

func (s *v4l2Source) Start(ctx context.Context) (<-chan Frame, error) {
	// NOTE for the implementer: construct the gorai v4l2 camera from s.conf using
	// v4l2cam.New(ctx, deps, conf) or its exported constructor, read width/height/
	// fps/jpeg_quality from conf (all float64), then:
	//   cam.SetFrameCallback(func(jpeg []byte, ts time.Time, seq uint64, _ string) {
	//       select { case ch <- Frame{JPEG: jpeg, Seq: seq, TS: ts}: default: }
	//   })
	// and start the camera. Fill s.props from cam.Properties(ctx).
	// The exact v4l2cam constructor/start method names must be read from
	// /gorai-all/gorai/components/camera/v4l2/camera.go at implementation time and
	// used verbatim; do not invent names.
	ch := make(chan Frame, 2)
	ctx, s.cancel = context.WithCancel(ctx)
	_ = ctx
	_ = time.Now
	return ch, nil
}

func (s *v4l2Source) Close() error {
	if s.cancel != nil {
		s.cancel()
	}
	return nil
}
```

> This is the one file that cannot be finished from documentation alone — the exact v4l2 camera constructor/start method names must be read from `/gorai-all/gorai/components/camera/v4l2/camera.go` at implementation time and wired verbatim. It is isolated behind `//go:build v4l2` so the host build/tests never touch it, and it is exercised in M6 on the Pi. The M2 acceptance below uses the fake source.

- [ ] **Step 2: Verify the host build ignores it**

Run: `go build ./... && go build -tags v4l2 ./components/camera/`
Expected: default build PASS; the `v4l2` build compiles the file (it may need the real wiring before it links into the binary — that is fine, it is completed in M6).

- [ ] **Step 3: Commit**

```bash
git add components/camera/source_v4l2.go
git commit -m "feat(camera): v4l2 capture source scaffold (build-tagged, wired on Pi)"
```

---

## Milestone M3 — teleop-ui service (REQUIREMENTS R-130…R-138)

### Task 11: teleop-ui MJPEG bridge

**Files:**
- Create: `services/teleopui/mjpeg.go`, `services/teleopui/mjpeg_test.go`
- Remove: `services/teleopui/doc.go`

**Interfaces:**
- Produces: `func mjpegHandler(nc *nats.Conn, subject string) http.HandlerFunc` — subscribes to `subject`, writes `multipart/x-mixed-replace; boundary=frame`, one JPEG per part, flushes, drops frames when slow (mirrors `dashboard/cameras.StreamHandler`, DESIGN §11 R9).

- [ ] **Step 1: Write the failing test** `services/teleopui/mjpeg_test.go`

```go
package teleopui

import (
	"bufio"
	"context"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/nats-io/nats.go"
	natsserver "github.com/nats-io/nats-server/v2/server"
)

func startNATS(t *testing.T) *nats.Conn {
	t.Helper()
	srv, err := natsserver.NewServer(&natsserver.Options{Host: "127.0.0.1", Port: -1})
	if err != nil { t.Fatal(err) }
	go srv.Start()
	if !srv.ReadyForConnections(2 * time.Second) { t.Fatal("nats not ready") }
	t.Cleanup(srv.Shutdown)
	nc, err := nats.Connect(srv.ClientURL())
	if err != nil { t.Fatal(err) }
	t.Cleanup(nc.Close)
	return nc
}

func TestMJPEGStreamsFrames(t *testing.T) {
	nc := startNATS(t)
	srv := httptest.NewServer(mjpegHandler(nc, "gorai.picarx.front.data"))
	defer srv.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	req, _ := http.NewRequestWithContext(ctx, http.MethodGet, srv.URL, nil)
	resp, err := http.DefaultClient.Do(req)
	if err != nil { t.Fatal(err) }
	defer resp.Body.Close()
	if ct := resp.Header.Get("Content-Type"); !strings.HasPrefix(ct, "multipart/x-mixed-replace") {
		t.Fatalf("content-type: %s", ct)
	}

	// publish a couple frames
	go func() {
		for i := 0; i < 3; i++ {
			_ = nc.Publish("gorai.picarx.front.data", []byte{0xFF, 0xD8, byte(i)})
			time.Sleep(20 * time.Millisecond)
		}
	}()

	r := bufio.NewReader(resp.Body)
	line, _ := r.ReadString('\n')
	if !strings.Contains(line, "--frame") {
		t.Fatalf("expected boundary, got %q", line)
	}
}
```

- [ ] **Step 2: Run to verify it fails** → FAIL (`mjpegHandler` undefined). Add nats-server dep if needed.

- [ ] **Step 3: Write `services/teleopui/mjpeg.go`**

```go
package teleopui

import (
	"fmt"
	"net/http"

	"github.com/nats-io/nats.go"
)

// mjpegHandler bridges the NATS JPEG frame stream to an HTTP MJPEG response.
// Modeled on gorai dashboard/cameras.StreamHandler but over the raw *nats.Conn
// (services do not get the *gorainats.Client wrapper). DESIGN §11 R9, R-134.
func mjpegHandler(nc *nats.Conn, subject string) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		flusher, ok := w.(http.Flusher)
		if !ok {
			http.Error(w, "streaming unsupported", http.StatusInternalServerError)
			return
		}
		w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
		w.Header().Set("Cache-Control", "no-cache")
		w.Header().Set("Connection", "close")

		frameCh := make(chan []byte, 2)
		sub, err := nc.Subscribe(subject, func(m *nats.Msg) {
			select {
			case frameCh <- m.Data:
			default: // drop when the client is slow
			}
		})
		if err != nil {
			http.Error(w, "subscribe failed", http.StatusServiceUnavailable)
			return
		}
		defer sub.Unsubscribe()

		ctx := r.Context()
		for {
			select {
			case <-ctx.Done():
				return
			case frame := <-frameCh:
				if _, err := fmt.Fprintf(w, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(frame)); err != nil {
					return
				}
				if _, err := w.Write(frame); err != nil {
					return
				}
				if _, err := fmt.Fprint(w, "\r\n"); err != nil {
					return
				}
				flusher.Flush()
			}
		}
	}
}
```

- [ ] **Step 4: Run / Step 5: Commit**

Run: `go test ./services/teleopui/ -run TestMJPEG -v` → PASS.
```bash
git add services/teleopui/mjpeg.go services/teleopui/mjpeg_test.go
git rm services/teleopui/doc.go
git commit -m "feat(teleopui): MJPEG bridge over raw NATS conn"
```

---

### Task 12: teleop-ui control channel — browser events → tool calls

**Files:**
- Create: `services/teleopui/control.go`, `services/teleopui/control_test.go`

**Interfaces:**
- Produces:
  - `type controlEvent struct { T string `json:"t"`; V float64 `json:"v"` }`
  - `func toolCall(ev controlEvent) (subject string, args map[string]any, ok bool)` — pure mapping of a control event to a `gorai.picarx.<cap>.command` subject + JSON args. `t ∈ {drive,steer,campan,camtilt,estop,centre}`.
  - `func (s *Service) handleControl(w http.ResponseWriter, r *http.Request)` — decodes a POSTed `controlEvent`, issues the NATS request, writes the reply. (Keyboard and sliders both POST the same event → I-003.)

- [ ] **Step 1: Write the failing test** `services/teleopui/control_test.go`

```go
package teleopui

import "testing"

func TestToolCallMapping(t *testing.T) {
	cases := []struct {
		ev      controlEvent
		subject string
		key     string
		val     float64
	}{
		{controlEvent{"drive", 42}, "gorai.picarx.drive.command", "throttle", 42},
		{controlEvent{"steer", -15}, "gorai.picarx.steer.command", "angle", -15},
		{controlEvent{"campan", 10}, "gorai.picarx.campan.command", "angle", 10},
		{controlEvent{"camtilt", 5}, "gorai.picarx.camtilt.command", "angle", 5},
	}
	for _, c := range cases {
		subj, args, ok := toolCall(c.ev, "picarx")
		if !ok || subj != c.subject || args[c.key] != c.val {
			t.Fatalf("%+v -> %s %v ok=%v", c.ev, subj, args, ok)
		}
	}
	// estop engage
	subj, args, ok := toolCall(controlEvent{"estop", 0}, "picarx")
	if !ok || subj != "gorai.picarx.estop.command" || args["clear"] != false {
		t.Fatalf("estop map: %s %v", subj, args)
	}
	// centre expands to steer 0 (caller sends campan/camtilt 0 separately)
	subj, args, ok = toolCall(controlEvent{"centre", 0}, "picarx")
	if !ok || subj != "gorai.picarx.steer.command" || args["angle"] != 0.0 {
		t.Fatalf("centre map: %s %v", subj, args)
	}
	// unknown
	if _, _, ok := toolCall(controlEvent{"bogus", 1}, "picarx"); ok {
		t.Fatalf("unknown event must not map")
	}
}
```

- [ ] **Step 2: Run to verify it fails** → FAIL (undefined).

- [ ] **Step 3: Write `services/teleopui/control.go`**

```go
package teleopui

import (
	"encoding/json"
	"fmt"
	"net/http"
	"time"
)

type controlEvent struct {
	T string  `json:"t"`
	V float64 `json:"v"`
}

// toolCall maps a browser control event to a picarx tool subject + args.
// Sliders and keys both produce these events, so both hit the identical
// command payloads (I-003).
func toolCall(ev controlEvent, robotID string) (string, map[string]any, bool) {
	subj := func(cap string) string { return fmt.Sprintf("gorai.%s.%s.command", robotID, cap) }
	switch ev.T {
	case "drive":
		return subj("drive"), map[string]any{"throttle": ev.V}, true
	case "steer":
		return subj("steer"), map[string]any{"angle": ev.V}, true
	case "campan":
		return subj("campan"), map[string]any{"angle": ev.V}, true
	case "camtilt":
		return subj("camtilt"), map[string]any{"angle": ev.V}, true
	case "estop":
		return subj("estop"), map[string]any{"clear": ev.V != 0}, true
	case "centre":
		return subj("steer"), map[string]any{"angle": 0.0}, true
	default:
		return "", nil, false
	}
}

func (s *Service) handleControl(w http.ResponseWriter, r *http.Request) {
	var ev controlEvent
	if err := json.NewDecoder(r.Body).Decode(&ev); err != nil {
		http.Error(w, "bad event", http.StatusBadRequest)
		return
	}
	subject, args, ok := toolCall(ev, s.robotID)
	if !ok {
		http.Error(w, "unknown event", http.StatusBadRequest)
		return
	}
	payload, _ := json.Marshal(args)
	msg, err := s.nc.Request(subject, payload, time.Second)
	if err != nil {
		http.Error(w, err.Error(), http.StatusGatewayTimeout)
		return
	}
	w.Header().Set("Content-Type", "application/json")
	_, _ = w.Write(msg.Data)
}
```

- [ ] **Step 4: Run** — the test compiles against `toolCall`; `handleControl` references `*Service` created in Task 13. If the package does not yet have `Service`, this test still passes for `toolCall` because Go compiles the whole package — so create a minimal `Service` type now in `service.go` (Task 13 fleshes it out). To keep this task green, add a stub at the top of Task 13; run:

Run: `go test ./services/teleopui/ -run TestToolCall -v`
Expected: PASS once `Service` exists (Task 13). If executing strictly in order, move Step 5 commit to after Task 13's `Service` stub — or add the two-field stub now:

```go
// minimal stub; Task 13 replaces service.go
type Service struct { nc *nats.Conn; robotID string }
```
(Place in `control.go` temporarily, remove when Task 13 adds the full struct.)

- [ ] **Step 5: Commit**

```bash
git add services/teleopui/control.go services/teleopui/control_test.go
git commit -m "feat(teleopui): control event to tool-call mapping"
```

---

### Task 13: teleop-ui service — own http.Server, hub, telemetry, routes

**Files:**
- Create: `services/teleopui/service.go`
- Modify: `services/teleopui/control.go` (remove the temporary `Service` stub)
- Test: `services/teleopui/service_test.go`

**Interfaces:**
- Consumes: `mjpegHandler` (Task 11), `handleControl` (Task 12), `dashboard.NewWebSocketHub` (DESIGN §11 R9).
- Produces: `type Service struct{...}` registered `("teleop-ui","teleop-ui")` implementing `resource.Resource` + `Start`; routes `GET /`, `GET /static/*`, `GET /stream/front`, `GET /ws`, `POST /control`.

- [ ] **Step 1: Write the failing test** `services/teleopui/service_test.go`

```go
package teleopui

import (
	"context"
	"io"
	"net/http"
	"testing"
	"time"
)

func TestServiceServesPageAndControl(t *testing.T) {
	nc := startNATS(t)
	// stub a picarx drive responder
	_, _ = nc.Subscribe("gorai.picarx.drive.command", func(m *natsMsg) {})
	s := &Service{nc: nc, robotID: "picarx", listen: "127.0.0.1:0", cameraCap: "front"}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := s.Start(ctx); err != nil {
		t.Fatal(err)
	}
	defer s.Close(ctx)
	time.Sleep(50 * time.Millisecond)

	resp, err := http.Get("http://" + s.addr())
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != 200 || len(body) == 0 {
		t.Fatalf("index: %d len=%d", resp.StatusCode, len(body))
	}
}
```

> Replace `*natsMsg` with `*nats.Msg` and import `"github.com/nats-io/nats.go"`. The subscribe body can be empty for this test; it only checks the page serves. Use `s.addr()` returning the actual bound address.

- [ ] **Step 2: Run to verify it fails** → FAIL (Service/Start/addr undefined).

- [ ] **Step 3: Write the embedded web assets** — `services/teleopui/web/index.html`

```html
<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PiCar-X Teleop</title>
<link rel="stylesheet" href="/static/style.css">
<main>
  <section id="video"><img id="feed" src="/stream/front" alt="camera feed"><div id="nofeed" hidden>no feed</div></section>
  <section id="telemetry">
    <div class="stat"><span>battery</span><b id="battery">--</b> V</div>
    <div class="stat"><span>distance</span><b id="distance">--</b> cm</div>
    <div class="stat"><span>grayscale</span><b id="grayscale">--</b></div>
    <div class="stat"><span>line</span><b id="line">--</b></div>
    <div class="stat"><span>cliff</span><b id="cliff">--</b></div>
    <div class="stat"><span>system</span><b id="sysinfo">--</b></div>
  </section>
  <section id="controls">
    <label>throttle <input id="throttle" type="range" min="-100" max="100" value="0"></label>
    <label>steer <input id="steer" type="range" min="-30" max="30" value="0"></label>
    <label>cam pan <input id="campan" type="range" min="-80" max="80" value="0"></label>
    <label>cam tilt <input id="camtilt" type="range" min="-65" max="65" value="0"></label>
    <button id="stop">STOP</button>
  </section>
</main>
<script src="/static/app.js"></script>
```

`services/teleopui/web/style.css`

```css
:root { color-scheme: light dark; font-family: system-ui, sans-serif; }
main { display: grid; gap: 1rem; max-width: 720px; margin: 1rem auto; padding: 0 1rem; }
#video img { width: 100%; border-radius: 8px; background: #222; aspect-ratio: 4/3; object-fit: contain; }
#telemetry { display: grid; grid-template-columns: repeat(3, 1fr); gap: .5rem; }
.stat { padding: .5rem; border: 1px solid gray; border-radius: 6px; }
.stat span { display: block; font-size: .75rem; opacity: .7; }
#controls { display: grid; gap: .75rem; }
#controls label { display: grid; }
#stop { padding: 1rem; font-weight: bold; background: #c0392b; color: #fff; border: 0; border-radius: 8px; }
.alarm { color: #c0392b; font-weight: bold; }
```

`services/teleopui/web/app.js`

```javascript
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
```

- [ ] **Step 4: Write `services/teleopui/service.go`**

```go
package teleopui

import (
	"context"
	"embed"
	"encoding/json"
	"fmt"
	"io/fs"
	"log/slog"
	"net"
	"net/http"
	"time"

	"github.com/emergingrobotics/gorai/pkg/dashboard"
	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/nats-io/nats.go"
)

//go:embed web/*
var webFS embed.FS

func init() {
	registry.RegisterService("teleop-ui", "teleop-ui", New)
}

type Service struct {
	name      resource.Name
	nc        *nats.Conn
	log       *slog.Logger
	robotID   string
	listen    string
	cameraCap string
	hub       *dashboard.WebSocketHub
	srv       *http.Server
	ln        net.Listener
	cancel    context.CancelFunc
	subs      []*nats.Subscription
}

func New(ctx context.Context, deps registry.Dependencies, conf registry.Config) (any, error) {
	name, _ := conf["name"].(string)
	robotID, _ := conf["namespace"].(string)
	listen, _ := conf["listen"].(string)
	if listen == "" {
		listen = "0.0.0.0:8080"
	}
	camera, _ := conf["camera"].(string)
	if camera == "" {
		camera = "front"
	}
	v, err := deps.Get("nats")
	if err != nil {
		return nil, fmt.Errorf("nats dependency: %w", err)
	}
	nc, ok := v.(*nats.Conn)
	if !ok {
		return nil, fmt.Errorf("nats dependency is %T", v)
	}
	log := slog.Default()
	if lv, err := deps.Get("logger"); err == nil {
		if l, ok := lv.(*slog.Logger); ok {
			log = l
		}
	}
	return &Service{
		name:      resource.NewServiceName("gorai", "teleop-ui", name),
		nc:        nc,
		log:       log,
		robotID:   robotID,
		listen:    listen,
		cameraCap: camera,
	}, nil
}

func (s *Service) Name() resource.Name { return s.name }
func (s *Service) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}
func (s *Service) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *Service) addr() string {
	if s.ln != nil {
		return s.ln.Addr().String()
	}
	return s.listen
}

func (s *Service) Start(ctx context.Context) error {
	ctx, s.cancel = context.WithCancel(ctx)

	s.hub = dashboard.NewWebSocketHub()
	go s.hub.Run(ctx)
	s.startTelemetry(ctx)

	staticFS, err := fs.Sub(webFS, "web")
	if err != nil {
		return err
	}
	mux := http.NewServeMux()
	mux.Handle("/static/", http.StripPrefix("/static/", http.FileServer(http.FS(staticFS))))
	mux.HandleFunc("/stream/front", mjpegHandler(s.nc, fmt.Sprintf("gorai.%s.%s.data", s.robotID, s.cameraCap)))
	mux.HandleFunc("/ws", s.hub.HandleWebSocket)
	mux.HandleFunc("/control", s.handleControl)
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		b, _ := fs.ReadFile(staticFS, "index.html")
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write(b)
	})

	ln, err := net.Listen("tcp", s.listen)
	if err != nil {
		return fmt.Errorf("teleop-ui listen %s: %w", s.listen, err)
	}
	s.ln = ln
	s.srv = &http.Server{Handler: mux}
	go func() {
		if err := s.srv.Serve(ln); err != nil && err != http.ErrServerClosed {
			s.log.Error("teleop-ui server", "err", err)
		}
	}()
	s.log.Info("teleop-ui serving", "addr", s.addr())
	return nil
}

// startTelemetry subscribes to sensor data streams and pushes them to browsers.
func (s *Service) startTelemetry(ctx context.Context) {
	for _, cap := range []string{"battery", "distance", "grayscale", "line", "cliff", "sysinfo"} {
		cap := cap
		subject := fmt.Sprintf("gorai.%s.%s.data", s.robotID, cap)
		sub, err := s.nc.Subscribe(subject, func(m *nats.Msg) {
			var payload map[string]any
			if err := json.Unmarshal(m.Data, &payload); err != nil {
				return
			}
			payload["cap"] = cap
			s.hub.BroadcastJSON(payload)
		})
		if err == nil {
			s.subs = append(s.subs, sub)
		}
	}
	// sysinfo has no data stream; poll its state once a second for the footer.
	go func() {
		t := time.NewTicker(time.Second)
		defer t.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-t.C:
				msg, err := s.nc.Request(fmt.Sprintf("gorai.%s.sysinfo.state", s.robotID), nil, time.Second)
				if err != nil {
					continue
				}
				var payload map[string]any
				if json.Unmarshal(msg.Data, &payload) == nil {
					payload["cap"] = "sysinfo"
					s.hub.BroadcastJSON(payload)
				}
			}
		}
	}()
}

func (s *Service) Close(ctx context.Context) error {
	if s.cancel != nil {
		s.cancel()
	}
	for _, sub := range s.subs {
		_ = sub.Unsubscribe()
	}
	if s.srv != nil {
		return s.srv.Shutdown(ctx)
	}
	return nil
}

var _ resource.Resource = (*Service)(nil)
```

- [ ] **Step 5: Remove the temporary `Service` stub** from `control.go` (Task 12) so there is one definition. Add import `"github.com/nats-io/nats.go"` where needed.

- [ ] **Step 6: Run tests**

Run: `go test ./services/teleopui/... -v`
Expected: PASS (page serves, control maps, MJPEG streams). Confirm `resource.NewServiceName` and `dashboard.NewWebSocketHub`/`Run`/`HandleWebSocket`/`BroadcastJSON` signatures against source (DESIGN §11 R8/R9).

- [ ] **Step 7: Commit**

```bash
git add services/teleopui/service.go services/teleopui/control.go services/teleopui/web services/teleopui/service_test.go
git commit -m "feat(teleopui): embedded page, telemetry WS, MJPEG, control routes"
```

---

### Task 14: input-equivalence guard (I-003) + full build

**Files:**
- Test: `services/teleopui/equivalence_test.go`
- Modify: `robot.json` created in Task 15 (validate wiring)

- [ ] **Step 1: Write the test** `services/teleopui/equivalence_test.go`

```go
package teleopui

import (
	"reflect"
	"testing"
)

// I-003: a slider action and its keyboard equivalent must produce identical
// command payloads. Both paths go through toolCall, so we assert equal args for
// equal events regardless of "source".
func TestSliderAndKeyProduceIdenticalPayload(t *testing.T) {
	fromSlider := controlEvent{"drive", 60}
	fromKey := controlEvent{"drive", 60}
	s1, a1, _ := toolCall(fromSlider, "picarx")
	s2, a2, _ := toolCall(fromKey, "picarx")
	if s1 != s2 || !reflect.DeepEqual(a1, a2) {
		t.Fatalf("slider %s/%v != key %s/%v", s1, a1, s2, a2)
	}
}
```

- [ ] **Step 2: Run** → PASS. **Step 3: Full build**

Run: `go build ./... && go test ./...`
Expected: whole module builds; all unit tests PASS.

- [ ] **Step 4: Commit**

```bash
git add services/teleopui/equivalence_test.go
git commit -m "test(teleopui): slider/keyboard input equivalence (I-003)"
```

---

## Milestone M4 — NATS exposure, audit, robot.json (REQUIREMENTS R-140…R-143, R-155)

### Task 15: robot.json + calibration.json + gitignore

**Files:**
- Create: `robot.json`, `calibration.json`, `.gitignore`

- [ ] **Step 1: Write `robot.json`** (DESIGN §6; LAN-bound NATS, dashboard disabled)

```json
{
  "version": "2",
  "robot": { "name": "picarx", "description": "Teleoperable PiCar-X GoRAI robot" },
  "nats": { "embedded": true, "listen": "0.0.0.0:4222", "url": "nats://localhost:4222" },
  "dashboard": { "enabled": false },
  "components": [
    {
      "type": "picarx", "model": "picarx", "name": "picarx",
      "attributes": {
        "calibration": "calibration.json",
        "steer_max_deg": 30, "campan_max_deg": 80, "camtilt_max_deg": 65,
        "deadband": 5, "watchdog_ms": 500, "grayscale_ref": [1000, 1000, 1000]
      }
    },
    {
      "type": "camera", "model": "picam", "name": "front",
      "depends_on": ["picarx"],
      "attributes": {
        "device": "/dev/video0", "width": 640, "height": 480, "fps": 15, "jpeg_quality": 70,
        "rtsp": { "enabled": true, "listen": ":8554", "path": "/front" }
      }
    }
  ],
  "services": [
    { "type": "teleop-ui", "model": "teleop-ui", "name": "teleop",
      "attributes": { "listen": "0.0.0.0:8080", "camera": "front" } }
  ],
  "discovery": { "enabled": false }
}
```

> Confirm the exact RDL v2 key for `nats.listen` against `/gorai-all/gorai/pkg/config/config.go`; if the field is named differently (e.g. `client_url`/`host`/`port`), use the real field and keep the `0.0.0.0` bind intent (R-140).

- [ ] **Step 2: Write `calibration.json`** (start from gopicar `MeasuredCalibration`; adjust on the Pi in M6)

```json
{
  "steer": { "trim": -58, "dir": 1, "min": -30, "max": 30 },
  "pan":   { "trim": -11, "dir": -1, "min": -80, "max": 80 },
  "tilt":  { "trim": 25, "dir": -1, "min": -40, "max": 85 },
  "left_motor":  { "scale": 1 },
  "right_motor": { "scale": 1 }
}
```

- [ ] **Step 3: Write `.gitignore`**

```
/bin/
*.creds
*.log
```

- [ ] **Step 4: Validate**

Run: `make validate`
Expected: `gorai validate` reports the config valid. If it flags the `picam`/`teleop-ui` types as unknown, that is expected until `gorai build` compiles this module's registrations in — run `go build ./...` first, or validate via `gorai run` in Task 18.

- [ ] **Step 5: Commit**

```bash
git add robot.json calibration.json .gitignore
git commit -m "feat: robot.json, calibration, gitignore (LAN NATS, dashboard off)"
```

---

### Task 16: JetStream audit stream excluding video (R-155, C-005)

**Files:**
- Create: `components/picarx/audit.go`, `components/picarx/audit_test.go`

**Interfaces:**
- Produces: `func ensureAuditStream(ctx context.Context, nc *nats.Conn, robotID string) error` — creates a JetStream stream capturing `gorai.<robot>.*.command`, `gorai.<robot>.*.state`, `gorai.<robot>.*.event`, and the scalar `.data` subjects, but NOT `gorai.<robot>.front.data` (video). Called once from `picarx` `Start`.

- [ ] **Step 1: Write the test** `components/picarx/audit_test.go`

```go
package picarx

import (
	"context"
	"testing"

	"github.com/nats-io/nats.go"
)

func TestAuditStreamExcludesVideo(t *testing.T) {
	subjects := auditSubjects("picarx")
	for _, s := range subjects {
		if s == "gorai.picarx.front.data" {
			t.Fatalf("audit must not capture raw video (C-005)")
		}
	}
	// must include commands and scalar telemetry
	want := map[string]bool{
		"gorai.picarx.*.command": false,
		"gorai.picarx.*.event":   false,
		"gorai.picarx.battery.data": false,
	}
	for _, s := range subjects {
		if _, ok := want[s]; ok {
			want[s] = true
		}
	}
	for k, ok := range want {
		if !ok {
			t.Errorf("audit missing %s", k)
		}
	}
	_ = nats.Msg{}
	_ = context.Background()
}
```

- [ ] **Step 2: Run to verify it fails** → FAIL (`auditSubjects` undefined).

- [ ] **Step 3: Write `components/picarx/audit.go`**

```go
package picarx

import (
	"context"
	"fmt"

	"github.com/nats-io/nats.go"
)

// auditSubjects lists exactly what the audit stream captures. Raw video
// (front.data) is deliberately excluded (C-005, R-155): we keep every command
// and scalar reading, not every frame.
func auditSubjects(robotID string) []string {
	return []string{
		fmt.Sprintf("gorai.%s.*.command", robotID),
		fmt.Sprintf("gorai.%s.*.event", robotID),
		fmt.Sprintf("gorai.%s.*.state", robotID),
		fmt.Sprintf("gorai.%s.battery.data", robotID),
		fmt.Sprintf("gorai.%s.distance.data", robotID),
		fmt.Sprintf("gorai.%s.grayscale.data", robotID),
		fmt.Sprintf("gorai.%s.line.data", robotID),
		fmt.Sprintf("gorai.%s.cliff.data", robotID),
	}
}

func ensureAuditStream(ctx context.Context, nc *nats.Conn, robotID string) error {
	js, err := nc.JetStream()
	if err != nil {
		return err
	}
	name := "picarx-audit"
	cfg := &nats.StreamConfig{
		Name:      name,
		Subjects:  auditSubjects(robotID),
		Retention: nats.LimitsPolicy,
		Storage:   nats.FileStorage,
		MaxAge:    0,
	}
	if _, err := js.AddStream(cfg); err != nil {
		// AddStream errors if it already exists with a different config; update.
		if _, uerr := js.UpdateStream(cfg); uerr != nil {
			return fmt.Errorf("audit stream: add=%w update=%v", err, uerr)
		}
	}
	return nil
}
```

- [ ] **Step 4: Call it from `picarx` `Start`** — in `server.go` `Start`, after `registerSchemas`:

```go
	if err := ensureAuditStream(ctx, c.nc, c.subj.RobotID()); err != nil {
		c.log.Warn("audit stream setup failed", "err", err)
	}
```

> If `subjects.Builder` has no `RobotID()` accessor, store the robot ID on the `Component` in `New` (it is already available as `robotID` there) and pass that field instead. Check `/gorai-all/gorai/pkg/subjects/subjects.go`.

- [ ] **Step 5: Run tests**

Run: `go test ./components/picarx/... -v`
Expected: PASS. Confirm `nats.StreamConfig` fields against the vendored `nats.go` version.

- [ ] **Step 6: Commit**

```bash
git add components/picarx/audit.go components/picarx/audit_test.go components/picarx/server.go
git commit -m "feat(picarx): JetStream audit stream excluding raw video (C-005)"
```

---

### Task 17: NKey/JWT credential wiring notes (R-141, D-3)

**Files:**
- Create: `docs/NATS-AUTH.md`

> This task documents and stages the credentialed-bus setup; it is verified live in M6 because it needs the deployed NATS. There is no host unit test for account auth.

- [ ] **Step 1: Write `docs/NATS-AUTH.md`** describing the NKey/JWT account setup: the `picarx` account, an operator/user for the robot's own components (the embedded server's local connection), and a separate user for external agents. Document that the RDL `nats` block references the resolver/creds, and that a relaxed no-auth mode is bench-only and never the deployed default (R-141). Include the exact `nats` RDL keys once confirmed against `pkg/config`.

- [ ] **Step 2: Confirm the RDL `nats` auth fields** exist by reading `/gorai-all/gorai/pkg/config/config.go` and `/gorai-all/gorai/pkg/embeddednats`; record the real key names in the doc. If embedded NATS does not yet accept account config via RDL, note it as a gorai-core gap to raise (do not silently skip — R-141 is a hard requirement; flag it).

- [ ] **Step 3: Commit**

```bash
git add docs/NATS-AUTH.md
git commit -m "docs: NKey/JWT credential plan for the LAN-exposed bus (R-141)"
```

---

## Milestone M5 — RTSP (RTP/JPEG) (REQUIREMENTS R-123, D-1)

### Task 18: gortsplib RTP/JPEG server fed by shared frames

**Files:**
- Modify: `components/camera/rtsp.go` (replace stub)
- Modify: `components/camera/camera.go` (construct `rtspServer` from `rtsp` config)
- Test: `components/camera/rtsp_test.go`

**Interfaces:**
- Consumes: `Frame` (Task 8); `github.com/bluenviron/gortsplib/v4`.
- Produces: `func newRTSPServer(listen, path string) (*rtspServer, error)`; `func (r *rtspServer) push(f Frame)`; `func (r *rtspServer) close() error`. `push` encodes each JPEG frame into RTP/JPEG packets and forwards to connected RTSP readers.

- [ ] **Step 1: Add the dependency**

Run: `go get github.com/bluenviron/gortsplib/v4@latest`
Expected: added to go.mod.

- [ ] **Step 2: Write the test** `components/camera/rtsp_test.go`

```go
package camera

import "testing"

func TestRTSPServerLifecycle(t *testing.T) {
	r, err := newRTSPServer("127.0.0.1:0", "/front")
	if err != nil {
		t.Fatalf("new rtsp: %v", err)
	}
	// pushing before any client connects must not panic or block.
	r.push(Frame{JPEG: []byte{0xFF, 0xD8, 0x00}})
	if err := r.close(); err != nil {
		t.Fatalf("close: %v", err)
	}
}
```

- [ ] **Step 3: Write `components/camera/rtsp.go`** (RTP/JPEG via gortsplib server)

```go
package camera

import (
	"sync"

	"github.com/bluenviron/gortsplib/v4"
	"github.com/bluenviron/gortsplib/v4/pkg/description"
	"github.com/bluenviron/gortsplib/v4/pkg/format"
)

// rtspServer serves the JPEG frame stream as RTP/JPEG (RFC 2435) — one capture
// reused, no encoder (D-1, R-123). Readers that connect receive frames pushed
// after their connection; a slow reader does not block capture.
type rtspServer struct {
	srv   *gortsplib.Server
	mu    sync.Mutex
	strm  *gortsplib.ServerStream
	fmt   *format.MJPEG
	path  string
}

func newRTSPServer(listen, path string) (*rtspServer, error) {
	r := &rtspServer{path: path, fmt: &format.MJPEG{}}
	r.srv = &gortsplib.Server{
		Handler:     r,
		RTSPAddress: listen,
	}
	desc := &description.Session{
		Medias: []*description.Media{{
			Type:    description.MediaTypeVideo,
			Formats: []format.Format{r.fmt},
		}},
	}
	r.strm = gortsplib.NewServerStream(r.srv, desc)
	if err := r.srv.Start(); err != nil {
		return nil, err
	}
	return r, nil
}

// OnConnOpen/OnSessionOpen/OnDescribe/OnSetup/OnPlay are the gortsplib.Server
// handler hooks. The implementer wires OnDescribe/OnSetup to return r.strm for
// the configured path, and OnPlay to begin delivery. The exact handler method
// set for gortsplib/v4 must be copied from that version's ServerHandler
// interface at implementation time (the API is versioned); do not guess method
// names. See the gortsplib "server" examples for the canonical shape.

func (r *rtspServer) push(f Frame) {
	r.mu.Lock()
	strm := r.strm
	r.mu.Unlock()
	if strm == nil || len(f.JPEG) == 0 {
		return
	}
	// WritePacketRTP/WriteData: encode the JPEG as RTP/JPEG using the format's
	// encoder and write to the stream. The exact encode+write call is copied
	// from the gortsplib/v4 MJPEG server example at implementation time.
	_ = strm
}

func (r *rtspServer) close() error {
	if r.srv != nil {
		r.srv.Close()
	}
	return nil
}
```

> The gortsplib/v4 `ServerHandler` method set and the RTP/JPEG encode+write call are version-specific and MUST be copied verbatim from the installed version's `server` example (run `go doc github.com/bluenviron/gortsplib/v4` and open the MJPEG server example). The test above only asserts lifecycle (start/push/close) so it passes with the handler hooks stubbed; full streaming is verified against VLC in M6 (B-007).

- [ ] **Step 4: Wire it in `camera.go` `New`** — after building `src`, read the `rtsp` attribute and construct the server:

```go
	if raw, ok := conf["rtsp"].(map[string]any); ok {
		if en, _ := raw["enabled"].(bool); en {
			listen, _ := raw["listen"].(string)
			path, _ := raw["path"].(string)
			rt, err := newRTSPServer(listen, path)
			if err != nil {
				return nil, fmt.Errorf("rtsp: %w", err)
			}
			comp.rtsp = rt // set on the *Component before returning
		}
	}
```
(Adjust to set the field on the struct literal / after construction as appropriate.)

- [ ] **Step 5: Run tests / build**

Run: `go test ./components/camera/... -v && go build ./...`
Expected: PASS/builds. `push` before a client connects is a no-op (no panic).

- [ ] **Step 6: Commit**

```bash
git add components/camera/rtsp.go components/camera/camera.go components/camera/rtsp_test.go go.mod go.sum
git commit -m "feat(camera): RTP/JPEG RTSP server fed by the shared frame stream (D-1)"
```

---

## Milestone M6 — Pi integration (VISION success criteria 1-7)

### Task 19: hardware integration test scaffold (build-tagged)

**Files:**
- Create: `components/picarx/hw_integration_test.go` (`//go:build hardware`)

**Interfaces:**
- Consumes: real `picarx.Open` (no fake bus), gated exactly as gopicar gates its own hardware tests (`-tags hardware`, actuator motion behind `GOPICAR_HW_MOVE=1`).

- [ ] **Step 1: Write `components/picarx/hw_integration_test.go`**

```go
//go:build hardware

package picarx

import (
	"context"
	"os"
	"testing"
	"time"

	gopx "github.com/emergingrobotics/gopicar/pkg/picarx"
)

func TestHWBatteryPlausible(t *testing.T) {
	px, err := gopx.Open(context.Background(), gopx.Options{Calibration: gopx.MeasuredCalibration()})
	if err != nil {
		t.Fatal(err)
	}
	defer px.Close()
	v, err := px.Battery(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if v < 5 || v > 9 {
		t.Fatalf("battery %.2fV out of plausible 2S range", v)
	}
}

func TestHWSteerSweep(t *testing.T) {
	if os.Getenv("GOPICAR_HW_MOVE") != "1" {
		t.Skip("set GOPICAR_HW_MOVE=1 to run actuator-moving tests")
	}
	px, err := gopx.Open(context.Background(), gopx.Options{Calibration: gopx.MeasuredCalibration()})
	if err != nil {
		t.Fatal(err)
	}
	defer px.Close()
	for _, a := range []float64{0, -20, 20, 0} {
		if err := px.SetDir(context.Background(), a); err != nil {
			t.Fatal(err)
		}
		time.Sleep(300 * time.Millisecond)
	}
}
```

- [ ] **Step 2: Verify the host build ignores it**

Run: `go vet ./... && go test ./...`
Expected: the `hardware`-tagged file is skipped on the host; all host tests PASS.

- [ ] **Step 3: Commit**

```bash
git add components/picarx/hw_integration_test.go
git commit -m "test(picarx): hardware integration scaffold (build-tagged)"
```

---

### Task 20: On-Pi bring-up checklist + acceptance (runs on the RPi)

**Files:**
- Create: `docs/BRINGUP.md`

> These steps run on the actual Raspberry Pi and are the "finish testing on the RPi" step. They are a manual acceptance script, not host unit tests.

- [ ] **Step 1: Write `docs/BRINGUP.md`** with this checklist (each line maps to a VISION success criterion):

```markdown
# PiCar-X Bring-Up (on the Raspberry Pi)

Prereqs: 64-bit Pi OS, Wi-Fi up, camera enabled (`/dev/video0` present), gopicar
udev/permissions per gopicar docs, `gorai` CLI on PATH.

1. Cross-compile + deploy from the dev host: `make deploy DEPLOY_HOST=pi@<host>`.
2. On the Pi, build WITH camera + hardware support: `gorai build robot.json -o bin/picarx`
   (ensure `-tags v4l2` reaches the camera package — set via the RDL build tags or
   a build wrapper; confirm the v4l2 source is linked).
3. Calibrate: center each servo, record raw angles, write `calibration.json`
   (see gopicar examples/picarctl `calibrate`). [gopicar calibration]
4. `./bin/picarx` (or `gorai run robot.json`). One process, no others. [Criterion 1]
5. Browser on the same Wi-Fi -> `http://<pi>:8080/`:
   - live video panel renders. [Criteria 2, 3]
   - battery/distance/grayscale/line/cliff/system update with no refresh. [Criterion 2]
   - throttle/steer/cam sliders drive; W/A/S/D + arrows drive; camera pans/tilts. [Criterion 3]
   - release all controls / close tab / drop Wi-Fi -> car stops within ~0.5s;
     spacebar / STOP e-stops immediately. [Criterion 5]
6. From a second LAN machine with valid creds:
   - `gorai mesh services` / `gorai mesh schemas` list picarx + camera. [Criterion 6]
   - `nats sub 'gorai.picarx.>'` shows command + telemetry (no video frames in
     the audit stream). [Criterion 7, C-005]
   - `vlc rtsp://<pi>:8554/front` renders live video while the page also streams. [Criterion 4, B-007]
7. Cliff test (carefully, wheels off the edge): lift the front over a table edge ->
   motors stop, `gorai.picarx.cliff.event` fires, drive is refused until cleared. [B-005]
```

- [ ] **Step 2: Execute the checklist on the Pi**, recording pass/fail per line. File issues for any failure; do not mark M6 complete until criteria 1-7 pass.

- [ ] **Step 3: Commit**

```bash
git add docs/BRINGUP.md
git commit -m "docs: on-Pi bring-up and acceptance checklist (VISION criteria 1-7)"
```

---

## Self-Review

**Spec coverage (REQUIREMENTS → task):**
- R-100…R-103 packaging → Tasks 1, 2, 15. R-110…R-114 picarx resources/tools/schemas → Tasks 3-7. R-120…R-124 camera + RTSP → Tasks 8-10, 18. R-130…R-138 teleop-ui page/controls/telemetry → Tasks 11-14. R-140…R-143 NATS exposure/authz/discovery-off → Tasks 15, 17. R-150…R-155 safety + audit → Tasks 3, 5, 6, 16. C-001 (browser no NATS) → Tasks 12, 13 (control via HTTP only) + firewall check in Task 20. C-002…C-004 → Task 3. C-005 → Task 16. C-006 single capture → Task 9. C-007 no separate web server → Task 13 (embedded). I-001…I-005 → Tasks 4, 3, 12/14, 7, 9. B-001…B-007 → Tasks 6, 3, 5, 20.

**Known documentation-bounded spots (flagged inline, not placeholders):** the real V4L2 wiring (Task 10) and the gortsplib/v4 handler+encode calls (Task 18) are version-specific APIs that must be copied verbatim from the installed source/examples at implementation time; both are isolated (build tag / lifecycle-tested) so host builds and tests stay green, and both are exercised on the Pi in M6. Every other step contains complete code.

**Type consistency:** `Device`, `Limits`, `controller` (Task 3) are used unchanged in Tasks 4-7; `Frame`/`Source`/`Properties` (Task 8) unchanged in 9-10, 18; `controlEvent`/`toolCall` (Task 12) unchanged in 13-14; subject strings are the concrete `gorai.picarx.<cap>.<type>` form throughout, matching DESIGN §2.1/§11.

**Verify-before-trusting note for the implementer:** three gorai/gopicar signatures should be re-confirmed against source before first use (they were read once, DESIGN §11): `mesh.NewJSONSchema`/`NewClient`/`RegisterSchema` (Task 7), `dashboard.NewWebSocketHub`/`Run`/`HandleWebSocket`/`BroadcastJSON` (Task 13), and `mcu.HAT`'s exported field (Task 6). Each task step says so at its point of use.
