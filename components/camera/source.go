package camera

import (
	"context"
	"time"
)

type Frame struct {
	JPEG []byte
	Seq  uint64
	TS   time.Time
}

type Properties struct {
	Width, Height int
	FrameRate     float64
	Encoding      string
	PTZ           bool
}

// Source is one camera capture. Exactly one Source is opened per camera (I-005).
type Source interface {
	Start(ctx context.Context) (<-chan Frame, error)
	Properties() Properties
	Close() error
}

type fakeSource struct {
	props  Properties
	frames [][]byte
	period time.Duration
}

func newFakeSource(props Properties, frames [][]byte, period time.Duration) Source {
	return &fakeSource{props: props, frames: frames, period: period}
}

func (f *fakeSource) Properties() Properties { return f.props }
func (f *fakeSource) Close() error           { return nil }

func (f *fakeSource) Start(ctx context.Context) (<-chan Frame, error) {
	ch := make(chan Frame)
	go func() {
		defer close(ch)
		t := time.NewTicker(f.period)
		defer t.Stop()
		var seq uint64
		i := 0
		for {
			select {
			case <-ctx.Done():
				return
			case now := <-t.C:
				jpeg := f.frames[i%len(f.frames)]
				i++
				seq++
				select {
				case ch <- Frame{JPEG: jpeg, Seq: seq, TS: now}:
				case <-ctx.Done():
					return
				}
			}
		}
	}()
	return ch, nil
}
