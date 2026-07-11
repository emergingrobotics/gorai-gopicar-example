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

	mu         sync.Mutex
	estopped   bool
	cliff      bool
	cliffClear int // consecutive clear readings; cliff clears after cliffClearPolls
	moving     bool
	lastDrive  time.Time
}

// cliffClearPolls is how many consecutive clear cliff readings are required
// before forward drive is re-enabled. At the ~100ms cliff poll this debounces a
// reading that flickers while the car teeters at an edge, which would otherwise
// briefly re-allow forward and let the car inch over. C-004.
const cliffClearPolls = 3

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
	clamped := clamp(throttle, -100, 100)
	// At a cliff, refuse FORWARD and force a stop, but allow stop/reverse so the
	// operator can back away from the edge (R-154, C-004).
	if c.cliff && clamped > c.lim.DriveDeadband {
		c.moving = false
		_ = c.dev.Stop(ctx)
		return failResp("cliff_blocked", "cliff detected; forward blocked — reverse to back away")
	}
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
// (which stops the motors). The cliff state latches on detection and clears only
// after cliffClearPolls consecutive clear readings, so a reading that flickers at
// the edge cannot momentarily re-enable forward drive. C-004.
func (c *controller) updateCliff(ctx context.Context, detected bool) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	rising := detected && !c.cliff
	if detected {
		c.cliff = true
		c.cliffClear = 0
		if rising {
			c.moving = false
			_ = c.dev.Stop(ctx)
		}
	} else if c.cliff {
		c.cliffClear++
		if c.cliffClear >= cliffClearPolls {
			c.cliff = false
			c.cliffClear = 0
		}
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
