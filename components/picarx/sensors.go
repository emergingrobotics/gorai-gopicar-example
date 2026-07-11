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
	go c.distanceLoop(ctx) // distance also feeds the proximity stop interlock
	go c.streamLoop(ctx, "grayscale", 100*time.Millisecond, c.grayscalePayload)
	go c.streamLoop(ctx, "line", 100*time.Millisecond, c.linePayload)

	// Snapshot state replies for each capability + sysinfo.
	states := []struct {
		name string
		fn   func(context.Context) map[string]any
	}{
		{"battery", c.batteryPayload},
		{"distance", c.distancePayload},
		{"grayscale", c.grayscalePayload},
		{"line", c.linePayload},
		{"cliff", c.cliffPayload},
		{"sysinfo", c.sysinfoPayload},
	}
	for _, s := range states {
		c.serveState(s.name, s.fn)
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

// distanceLoop reads the forward ultrasonic once per tick, publishes it to
// distance.data, and feeds the proximity stop interlock (a single read, so the
// slow sensor is not polled twice). On a rising edge into the stop zone it also
// publishes a proximity event.
func (c *Component) distanceLoop(ctx context.Context) {
	dataSubj := c.subj.ComponentData("distance")
	t := time.NewTicker(100 * time.Millisecond)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
			cm, err := c.ctl.dev.Distance(ctx, 30*time.Millisecond)
			if err != nil {
				continue
			}
			if b, err := json.Marshal(map[string]any{"cm": cm}); err == nil {
				_ = c.nc.Publish(dataSubj, b)
			}
			if c.ctl.updateProximity(ctx, cm) {
				payload, _ := json.Marshal(map[string]any{"obstacle": true, "cm": cm, "ts": time.Now().UTC().Format(time.RFC3339)})
				_ = c.nc.Publish(c.subj.ComponentEvent("proximity"), payload)
				c.log.Warn("proximity stop: obstacle within threshold", "cm", cm)
			}
		}
	}
}

func (c *Component) serveState(capability string, fn func(context.Context) map[string]any) {
	sub, err := c.nc.Subscribe(c.subj.ComponentState(capability), func(m *nats.Msg) {
		if b, err := json.Marshal(fn(context.Background())); err == nil {
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
	hat := c.ctl.dev.HAT()
	return map[string]any{
		"fw":      fmt.Sprintf("%d.%d.%d", maj, min, patch),
		"hat":     hat.UUID,
		"hat_ver": int(hat.Version),
		"addr":    int(c.ctl.dev.Addr()),
	}
}
