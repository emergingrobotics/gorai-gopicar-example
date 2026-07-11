package picarx

import (
	"context"
	"encoding/json"
	"testing"
	"time"
)

func TestBatteryDataPublishes(t *testing.T) {
	nc := startNATS(t)
	d := &stubDev{}
	c := newTestComponent(nc, d)

	sub, err := nc.SubscribeSync("gorai.picarx.battery.data")
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	c.startSensors(ctx)

	msg, err := sub.NextMsg(2 * time.Second)
	if err != nil {
		t.Fatalf("no battery data: %v", err)
	}
	var p map[string]any
	_ = json.Unmarshal(msg.Data, &p)
	if p["volts"] != 7.4 {
		t.Fatalf("battery payload: %v", p)
	}
}

func TestSysinfoStateReply(t *testing.T) {
	nc := startNATS(t)
	c := newTestComponent(nc, &stubDev{})
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	c.startSensors(ctx)

	msg, err := nc.Request("gorai.picarx.sysinfo.state", nil, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	var p map[string]any
	_ = json.Unmarshal(msg.Data, &p)
	if p["fw"] != "2.1.1" {
		t.Fatalf("sysinfo fw: %v", p)
	}
}
