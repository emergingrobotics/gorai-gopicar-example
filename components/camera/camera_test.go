package camera

import (
	"context"
	"testing"
	"time"

	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
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

func TestCameraPublishesFrames(t *testing.T) {
	nc := startNATS(t)
	c := &Component{
		name:    resource.NewComponentName("gorai", "camera", "front"),
		nc:      nc,
		subj:    subjects.NewBuilder("picarx"),
		capName: "front",
		src: newFakeSource(Properties{Width: 4, Height: 4, FrameRate: 50, Encoding: "jpeg"},
			[][]byte{{0xFF, 0xD8, 0x09}}, 5*time.Millisecond),
	}
	sub, err := nc.SubscribeSync("gorai.picarx.front.data")
	if err != nil {
		t.Fatal(err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := c.Start(ctx); err != nil {
		t.Fatal(err)
	}

	msg, err := sub.NextMsg(2 * time.Second)
	if err != nil {
		t.Fatalf("no frame: %v", err)
	}
	if len(msg.Data) != 3 || msg.Data[0] != 0xFF {
		t.Fatalf("unexpected frame bytes: %v", msg.Data)
	}
}
