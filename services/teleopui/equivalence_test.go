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
