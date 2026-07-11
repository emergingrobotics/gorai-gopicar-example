package picarx

import "testing"

func TestSchemaCatalogCoversSurface(t *testing.T) {
	got := map[string]bool{}
	for _, s := range allSchemas() {
		got[s.Name] = true
	}
	for _, want := range []string{
		"gorai.picarx.drive.command", "gorai.picarx.steer.command",
		"gorai.picarx.campan.command", "gorai.picarx.camtilt.command",
		"gorai.picarx.estop.command", "gorai.picarx.battery.data",
		"gorai.picarx.distance.data", "gorai.picarx.grayscale.data",
		"gorai.picarx.line.data", "gorai.picarx.cliff.data",
		"gorai.picarx.sysinfo.state",
	} {
		if !got[want] {
			t.Errorf("missing schema %s", want)
		}
	}
}
