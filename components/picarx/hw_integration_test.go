//go:build hardware

package picarx

import (
	"context"
	"os"
	"testing"
	"time"

	gopx "github.com/emergingrobotics/gopicar/pkg/picarx"
)

func TestHWBatteryPlausible(t *testing.T) {
	px, err := gopx.Open(context.Background(), gopx.Options{Calibration: gopx.MeasuredCalibration()})
	if err != nil {
		t.Fatal(err)
	}
	defer px.Close()
	v, err := px.Battery(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if v < 5 || v > 9 {
		t.Fatalf("battery %.2fV out of plausible 2S range", v)
	}
}

func TestHWSteerSweep(t *testing.T) {
	if os.Getenv("GOPICAR_HW_MOVE") != "1" {
		t.Skip("set GOPICAR_HW_MOVE=1 to run actuator-moving tests")
	}
	px, err := gopx.Open(context.Background(), gopx.Options{Calibration: gopx.MeasuredCalibration()})
	if err != nil {
		t.Fatal(err)
	}
	defer px.Close()
	for _, a := range []float64{0, -20, 20, 0} {
		if err := px.SetDir(context.Background(), a); err != nil {
			t.Fatal(err)
		}
		time.Sleep(300 * time.Millisecond)
	}
}
