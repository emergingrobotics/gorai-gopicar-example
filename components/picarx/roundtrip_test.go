package picarx

import (
	"context"
	"encoding/json"
	"testing"
	"time"

	natsserver "github.com/nats-io/nats-server/v2/server"
	"github.com/nats-io/nats.go"
)

// startNATS spins an in-process NATS server for round-trip tests.
func startNATS(t *testing.T) *nats.Conn {
	t.Helper()
	opts := &natsserver.Options{Host: "127.0.0.1", Port: -1, JetStream: true, StoreDir: t.TempDir()}
	srv, err := natsserver.NewServer(opts)
	if err != nil {
		t.Fatal(err)
	}
	go srv.Start()
	if !srv.ReadyForConnections(2 * time.Second) {
		t.Fatal("nats not ready")
	}
	t.Cleanup(srv.Shutdown)
	nc, err := nats.Connect(srv.ClientURL())
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(nc.Close)
	return nc
}

func TestDriveCommandRoundTrip(t *testing.T) {
	nc := startNATS(t)
	d := &stubDev{}
	c := newTestComponent(nc, d)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := c.Start(ctx); err != nil {
		t.Fatal(err)
	}

	req, _ := json.Marshal(map[string]any{"throttle": 40.0})
	msg, err := nc.Request("gorai.picarx.drive.command", req, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	var resp map[string]any
	_ = json.Unmarshal(msg.Data, &resp)
	if resp["ok"] != true {
		t.Fatalf("drive reply: %v", resp)
	}
	if d.fwd != 1 || d.lastFwd != 40 {
		t.Fatalf("expected Forward(40), got fwd=%d last=%v", d.fwd, d.lastFwd)
	}
}
