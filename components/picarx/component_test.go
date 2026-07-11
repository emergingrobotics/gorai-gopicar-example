package picarx

import (
	"log/slog"
	"testing"
	"time"

	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
	"github.com/nats-io/nats.go"
)

func TestParseConfigDefaultsAndOverrides(t *testing.T) {
	lim, ref, calib, win := parseConfig(map[string]any{
		"steer_max_deg": float64(25),
		"grayscale_ref": []any{float64(900), float64(950), float64(1000)},
		"calibration":   "calib.json",
		"watchdog_ms":   float64(400),
	})
	if lim.SteerMaxDeg != 25 {
		t.Fatalf("steer max override: %v", lim.SteerMaxDeg)
	}
	if lim.CamPanMaxDeg != defaultCamPanMaxDeg {
		t.Fatalf("campan default: %v", lim.CamPanMaxDeg)
	}
	if ref != [3]int{900, 950, 1000} {
		t.Fatalf("grayscale ref: %v", ref)
	}
	if calib != "calib.json" {
		t.Fatalf("calibration path: %v", calib)
	}
	if win != 400*time.Millisecond {
		t.Fatalf("watchdog window: %v", win)
	}
}

// newTestComponent builds a Component wired to a stub device (no gopicar.Open),
// for NATS round-trip tests (Tasks 5, 6).
func newTestComponent(nc *nats.Conn, d Device) *Component {
	ctl := newController(d, testLim, [3]int{}, time.Now)
	return &Component{
		name:    resource.NewComponentName("gorai", "picarx", "picarx"),
		nc:      nc,
		log:     slog.Default(),
		robotID: "picarx",
		subj:    subjects.NewBuilder("picarx"),
		ctl:     ctl,
		grayRef: [3]int{},
	}
}
