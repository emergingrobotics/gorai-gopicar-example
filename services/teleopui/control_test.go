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
	subj, args, ok := toolCall(controlEvent{"estop", 0}, "picarx")
	if !ok || subj != "gorai.picarx.estop.command" || args["clear"] != false {
		t.Fatalf("estop map: %s %v", subj, args)
	}
	subj, args, ok = toolCall(controlEvent{"centre", 0}, "picarx")
	if !ok || subj != "gorai.picarx.steer.command" || args["angle"] != 0.0 {
		t.Fatalf("centre map: %s %v", subj, args)
	}
	if _, _, ok := toolCall(controlEvent{"bogus", 1}, "picarx"); ok {
		t.Fatalf("unknown event must not map")
	}
}
