package camera

import (
	"context"
	"testing"
	"time"
)

func TestFakeSourceEmits(t *testing.T) {
	src := newFakeSource(Properties{Width: 4, Height: 4, FrameRate: 50, Encoding: "jpeg"},
		[][]byte{{0xFF, 0xD8, 0x01}, {0xFF, 0xD8, 0x02}}, 5*time.Millisecond)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	ch, err := src.Start(ctx)
	if err != nil {
		t.Fatal(err)
	}
	f1 := <-ch
	f2 := <-ch
	if f1.Seq == f2.Seq {
		t.Fatalf("seq must advance: %d %d", f1.Seq, f2.Seq)
	}
	if len(f1.JPEG) == 0 {
		t.Fatalf("empty frame")
	}
	if src.Properties().Width != 4 {
		t.Fatalf("props")
	}
}
