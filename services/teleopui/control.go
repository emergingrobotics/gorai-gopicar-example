package teleopui

import "fmt"

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
	case "spin":
		return subj("spin"), map[string]any{"rate": ev.V}, true
	case "campan":
		return subj("campan"), map[string]any{"angle": ev.V}, true
	case "camtilt":
		return subj("camtilt"), map[string]any{"angle": ev.V}, true
	case "estop":
		return subj("estop"), map[string]any{"clear": ev.V != 0}, true
	case "proximity":
		return subj("proximity"), map[string]any{"cm": ev.V}, true
	case "centre":
		return subj("steer"), map[string]any{"angle": 0.0}, true
	default:
		return "", nil, false
	}
}
