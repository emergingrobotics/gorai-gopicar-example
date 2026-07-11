package picarx

import (
	"context"

	"github.com/emergingrobotics/gorai/pkg/mesh"
)

// allSchemas is the discovery surface (DESIGN 11 R6): registered for tools/list
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
	add("gorai.picarx.spin.command", "spin in place rate -100..100 (>0 right)", obj(map[string]any{"rate": numProp}, "rate"))
	add("gorai.picarx.campan.command", "camera pan deg", obj(map[string]any{"angle": numProp}, "angle"))
	add("gorai.picarx.camtilt.command", "camera tilt deg", obj(map[string]any{"angle": numProp}, "angle"))
	add("gorai.picarx.estop.command", "engage/clear e-stop", obj(map[string]any{"clear": map[string]any{"type": "boolean"}}))
	add("gorai.picarx.proximity.command", "set forward stop distance cm (0 = query)", obj(map[string]any{"cm": numProp}))
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
