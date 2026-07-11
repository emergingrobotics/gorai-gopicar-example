package teleopui

import (
	"bufio"
	"context"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	natsserver "github.com/nats-io/nats-server/v2/server"
	"github.com/nats-io/nats.go"
)

func startNATS(t *testing.T) *nats.Conn {
	t.Helper()
	srv, err := natsserver.NewServer(&natsserver.Options{Host: "127.0.0.1", Port: -1})
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

func TestMJPEGStreamsFrames(t *testing.T) {
	nc := startNATS(t)
	srv := httptest.NewServer(mjpegHandler(nc, "gorai.picarx.front.data"))
	defer srv.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	req, _ := http.NewRequestWithContext(ctx, http.MethodGet, srv.URL, nil)
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	if ct := resp.Header.Get("Content-Type"); !strings.HasPrefix(ct, "multipart/x-mixed-replace") {
		t.Fatalf("content-type: %s", ct)
	}

	go func() {
		for i := 0; i < 3; i++ {
			_ = nc.Publish("gorai.picarx.front.data", []byte{0xFF, 0xD8, byte(i)})
			time.Sleep(20 * time.Millisecond)
		}
	}()

	r := bufio.NewReader(resp.Body)
	line, _ := r.ReadString('\n')
	if !strings.Contains(line, "--frame") {
		t.Fatalf("expected boundary, got %q", line)
	}
}
