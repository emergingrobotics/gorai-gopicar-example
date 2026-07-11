package picarx

import "testing"

func TestAuditStreamExcludesVideo(t *testing.T) {
	subjects := auditSubjects("picarx")
	for _, s := range subjects {
		if s == "gorai.picarx.front.data" {
			t.Fatalf("audit must not capture raw video (C-005)")
		}
	}
	want := map[string]bool{
		"gorai.picarx.audit.command": false,
		"gorai.picarx.*.event":       false,
		"gorai.picarx.battery.data":  false,
	}
	for _, s := range subjects {
		if _, ok := want[s]; ok {
			want[s] = true
		}
	}
	for k, ok := range want {
		if !ok {
			t.Errorf("audit missing %s", k)
		}
	}
}
