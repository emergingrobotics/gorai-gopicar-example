package picarx

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"sync"
	"syscall"
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
	name     resource.Name
	nc       *nats.Conn
	log      *slog.Logger
	robotID  string
	subj     *subjects.Builder
	px       *gopx.PiCarX
	ctl      *controller
	grayRef  [3]int
	cancel   context.CancelFunc
	subs     []*nats.Subscription
	stopOnce sync.Once
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

	comp := &Component{
		name:    resource.NewComponentName("gorai", "picarx", name),
		nc:      nc,
		log:     log,
		robotID: robotID,
		subj:    subjects.NewBuilder(robotID),
		px:      px,
		ctl:     ctl,
		grayRef: ref,
	}
	comp.installExitHandler()
	return comp, nil
}

// installExitHandler guarantees the motors are cut and the HAT is reset on
// process exit. The gorai runtime also traps SIGINT/SIGTERM for graceful
// shutdown; signal.Notify fans out to both, so this fires an immediate hard stop
// the instant Ctrl-C/SIGTERM arrives, without waiting for the shutdown sequence
// to unwind to this component's Close. Cannot cover SIGKILL (uncatchable).
func (c *Component) installExitHandler() {
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigCh
		c.hardStop()
	}()
}

// hardStop cuts all motion and hard-resets the HAT/MCU so no PWM (motor or servo)
// stays latched after we exit. Runs at most once (Close and the signal handler
// both call it). Best-effort with a short independent timeout so a wedged bus
// cannot block exit.
func (c *Component) hardStop() {
	c.stopOnce.Do(func() {
		if c.px == nil {
			return
		}
		ctx, cancel := context.WithTimeout(context.Background(), time.Second)
		defer cancel()
		c.log.Warn("picarx hard stop: cutting motors and resetting HAT")
		if err := c.px.Stop(ctx); err != nil {
			c.log.Warn("hard stop: motor stop failed", "err", err)
		}
		if err := c.px.Reset(ctx); err != nil {
			c.log.Warn("hard stop: HAT reset failed", "err", err)
		}
	})
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
		c.hardStop() // stop motors + reset HAT BEFORE releasing the bus/GPIO
		return c.px.Close()
	}
	return nil
}

var (
	_ resource.Resource = (*Component)(nil)
	_ Device            = (*gopx.PiCarX)(nil)
)
