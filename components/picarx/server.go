package picarx

import (
	"context"
	"encoding/json"
	"fmt"
	"time"

	"github.com/nats-io/nats.go"
)

// Start wires the NATS command server, sensor publishers, watchdog and cliff
// loops. It is invoked by the robot runtime (Startable). NATS pub/sub lives here;
// safety decisions live in the controller.
func (c *Component) Start(ctx context.Context) error {
	ctx, c.cancel = context.WithCancel(ctx)

	// Tools (command request/reply). Args are JSON map[string]any (DESIGN 11 R4).
	c.serveCommand(ctx, "drive", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.drive(ctx, num(a, "throttle"))
	})
	c.serveCommand(ctx, "steer", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.steer(ctx, num(a, "angle"))
	})
	c.serveCommand(ctx, "spin", func(ctx context.Context, a map[string]any) map[string]any {
		return c.ctl.spin(ctx, num(a, "rate"))
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
	// proximity: {cm>0} sets the forward-obstacle stop distance; otherwise reports it.
	c.serveCommand(ctx, "proximity", func(ctx context.Context, a map[string]any) map[string]any {
		if cm, ok := a["cm"].(float64); ok && cm > 0 {
			return c.ctl.setProximity(cm)
		}
		return map[string]any{"ok": true, "cm": c.ctl.proximity()}
	})

	if err := c.registerSchemas(ctx); err != nil {
		c.log.Warn("schema registration failed", "err", err)
	}
	if err := ensureAuditStream(c.nc, c.robotID); err != nil {
		c.log.Warn("audit stream setup failed", "err", err)
	}

	c.centerServos(ctx)
	c.startSensors(ctx)
	go c.watchdogLoop(ctx)
	go c.cliffLoop(ctx)
	return nil
}

// centerServos moves steering and the camera gimbal to neutral (0) at startup so
// the robot always begins from a known pose (matches the GUI sliders, which
// default to centre). Routed through the controller (same clamped path as the
// tools) so it uses the injected device.
func (c *Component) centerServos(ctx context.Context) {
	servos := []struct {
		name string
		set  func(context.Context, float64) map[string]any
	}{
		{"steer", c.ctl.steer},
		{"campan", c.ctl.campan},
		{"camtilt", c.ctl.camtilt},
	}
	for _, s := range servos {
		if r := s.set(ctx, 0); r["ok"] != true {
			c.log.Warn("centre servo failed", "servo", s.name, "resp", r)
		}
	}
	c.log.Info("servos centred")
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
		// Audit the command + its reply on a dedicated subject the JetStream stream
		// captures (R-155). Not the .command subject itself — see auditSubjects.
		if b, err := json.Marshal(map[string]any{"cap": capability, "args": args, "resp": resp}); err == nil {
			_ = c.nc.Publish(fmt.Sprintf("gorai.%s.audit.command", c.robotID), b)
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
			payload, _ := json.Marshal(map[string]any{"cliff": detected})
			_ = c.nc.Publish(c.subj.ComponentData("cliff"), payload)
		}
	}
}
