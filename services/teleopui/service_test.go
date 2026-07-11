package teleopui

import (
	"context"
	"io"
	"log/slog"
	"net/http"
	"testing"
	"time"

	"github.com/nats-io/nats.go"
)

func TestServiceServesPageAndControl(t *testing.T) {
	nc := startNATS(t)
	// A picarx drive responder so /control has something to reply.
	_, _ = nc.Subscribe("gorai.picarx.drive.command", func(m *nats.Msg) {
		_ = m.Respond([]byte(`{"ok":true}`))
	})
	s := &Service{nc: nc, robotID: "picarx", listen: "127.0.0.1:0", cameraCap: "front", log: slog.Default()}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := s.Start(ctx); err != nil {
		t.Fatal(err)
	}
	defer s.Close(ctx)
	time.Sleep(50 * time.Millisecond)

	resp, err := http.Get("http://" + s.addr())
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != 200 || len(body) == 0 {
		t.Fatalf("index: %d len=%d", resp.StatusCode, len(body))
	}
}
