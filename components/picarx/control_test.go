package picarx

import (
	"context"
	"testing"
	"time"

	"github.com/emergingrobotics/gopicar/pkg/mcu"
)

// stubDev records calls and lets tests force errors. It satisfies Device.
type stubDev struct {
	fwd, back, stop, spin                int
	lastFwd, lastBack, lastDir, lastSpin float64
	err                                  error
}

func (s *stubDev) SetDir(_ context.Context, d float64) error  { s.lastDir = d; return s.err }
func (s *stubDev) SetCamPan(context.Context, float64) error   { return s.err }
func (s *stubDev) SetCamTilt(context.Context, float64) error  { return s.err }
func (s *stubDev) Forward(_ context.Context, p float64) error { s.fwd++; s.lastFwd = p; return s.err }
func (s *stubDev) Backward(_ context.Context, p float64) error {
	s.back++
	s.lastBack = p
	return s.err
}
func (s *stubDev) Spin(_ context.Context, p float64) error   { s.spin++; s.lastSpin = p; return s.err }
func (s *stubDev) Stop(context.Context) error                { s.stop++; return s.err }
func (s *stubDev) Battery(context.Context) (float64, error)  { return 7.4, nil }
func (s *stubDev) Grayscale(context.Context) ([3]int, error) { return [3]int{}, nil }
func (s *stubDev) Distance(context.Context, time.Duration) (float64, error) {
	return 12, nil
}
func (s *stubDev) LineStatus(context.Context, [3]int) ([3]bool, error) { return [3]bool{}, nil }
func (s *stubDev) CliffStatus(context.Context, [3]int) (bool, error)   { return false, nil }
func (s *stubDev) FirmwareVersion(context.Context) (uint8, uint8, uint8, error) {
	return 2, 1, 1, nil
}
func (s *stubDev) HAT() mcu.HAT { return mcu.HAT{UUID: "test-hat"} }
func (s *stubDev) Addr() uint8  { return 0x14 }

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
		t.Fatalf("drive forward over cliff must fail cliff_blocked, got %v", r)
	}
	// Reverse MUST be allowed while at a cliff so the operator can back away.
	if r := c.drive(context.Background(), -20); r["ok"] != true {
		t.Fatalf("reverse while at cliff must succeed, got %v", r)
	}
	if c.updateCliff(context.Background(), true) {
		t.Fatalf("no re-fire while still detected")
	}
	// A single clear reading must NOT re-enable forward (debounce); only after
	// cliffClearPolls consecutive clears.
	c.updateCliff(context.Background(), false)
	if r := c.drive(context.Background(), 20); r["error"] != "cliff_blocked" {
		t.Fatalf("one clear reading must not re-enable forward, got %v", r)
	}
	for i := 0; i < cliffClearPolls; i++ {
		c.updateCliff(context.Background(), false)
	}
	if r := c.drive(context.Background(), 20); r["ok"] != true {
		t.Fatalf("drive after cliff fully clears should succeed, got %v", r)
	}
}

// A forward obstacle within proximityStopCM stops and blocks forward; reverse is
// allowed to back away; forward re-enables only after the path is stably clear.
func TestProximityInterlock(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	if c.updateProximity(context.Background(), 30) {
		t.Fatalf("far reading must not fire")
	}
	if r := c.drive(context.Background(), 20); r["ok"] != true {
		t.Fatalf("drive with clear path should succeed, got %v", r)
	}
	if !c.updateProximity(context.Background(), 3) || d.stop < 1 {
		t.Fatalf("close reading must fire+Stop, stop=%d", d.stop)
	}
	if r := c.drive(context.Background(), 20); r["error"] != "obstacle_blocked" {
		t.Fatalf("forward near obstacle must fail obstacle_blocked, got %v", r)
	}
	if r := c.drive(context.Background(), -20); r["ok"] != true {
		t.Fatalf("reverse away from obstacle must succeed, got %v", r)
	}
	// A single clear reading (incl. -1 no-echo) must not re-enable forward (debounce).
	c.updateProximity(context.Background(), -1)
	if r := c.drive(context.Background(), 20); r["error"] != "obstacle_blocked" {
		t.Fatalf("one clear reading must not re-enable forward, got %v", r)
	}
	for i := 0; i < cliffClearPolls; i++ {
		c.updateProximity(context.Background(), 30)
	}
	if r := c.drive(context.Background(), 20); r["ok"] != true {
		t.Fatalf("forward should resume after path stably clear, got %v", r)
	}
}

// Spin drives the rear motors in opposite directions, clamps, and is refused
// while e-stopped.
func TestSpin(t *testing.T) {
	d := &stubDev{}
	c := newController(d, testLim, [3]int{}, time.Now)
	if r := c.spin(context.Background(), 60); r["ok"] != true || d.spin != 1 || d.lastSpin != 60 {
		t.Fatalf("spin right: resp=%v spin=%d last=%v", r, d.spin, d.lastSpin)
	}
	if r := c.spin(context.Background(), 200); r["clamped"] != 100.0 || d.lastSpin != 100 {
		t.Fatalf("spin must clamp to 100, resp=%v last=%v", r, d.lastSpin)
	}
	c.estop(context.Background(), false)
	if r := c.spin(context.Background(), 60); r["error"] != "estop_latched" {
		t.Fatalf("spin while e-stopped must fail, got %v", r)
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
