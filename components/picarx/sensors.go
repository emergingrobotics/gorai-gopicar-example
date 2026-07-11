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
