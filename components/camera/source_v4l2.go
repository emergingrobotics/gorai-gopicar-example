//go:build v4l2

// Package camera's real capture source, backed by gorai's V4L2 camera component.
// Built only with -tags v4l2 (on the Pi); the host build and tests use the fake
// source, so this file never touches hardware off-target.
package camera

import (
	"context"
	"fmt"
	"time"

	v4l2cam "github.com/emergingrobotics/gorai/components/camera/v4l2"
	"github.com/emergingrobotics/gorai/pkg/registry"
)

func init() {
	sourceFactory = func(conf registry.Config) (Source, error) {
		return &v4l2Source{conf: conf}, nil
	}
}

type v4l2Source struct {
	conf  registry.Config
	cam   *v4l2cam.Camera
	props Properties
}

func (s *v4l2Source) Properties() Properties { return s.props }

func (s *v4l2Source) Start(ctx context.Context) (<-chan Frame, error) {
	num := func(key string, def float64) float64 {
		if v, ok := s.conf[key].(float64); ok {
			return v
		}
		return def
	}
	device, _ := s.conf["device"].(string)
	if device == "" {
		device = "/dev/video0"
	}
	fps := num("fps", 15)

	// publish_to_bus is off: only OUR camera component publishes to NATS, on our
	// subject and with our single capture (I-005/C-006). We consume frames via the
	// V4L2 component's frame callback instead.
	v4l2conf := registry.Config{
		"name":            "front",
		"device":          device,
		"width":           num("width", 640),
		"height":          num("height", 480),
		"frame_rate":      fps,
		"jpeg_quality":    num("jpeg_quality", 70),
		"publish_to_bus":  false,
		"publish_rate_hz": fps,
	}

	// deps is unused by v4l2cam.New; nil is safe (verified against source).
	obj, err := v4l2cam.New(ctx, nil, v4l2conf)
	if err != nil {
		return nil, fmt.Errorf("v4l2 camera: %w", err)
	}
	cam, ok := obj.(*v4l2cam.Camera)
	if !ok {
		return nil, fmt.Errorf("v4l2 camera returned %T", obj)
	}
	s.cam = cam

	if p, err := cam.Properties(ctx); err == nil {
		s.props = Properties{Width: p.Width, Height: p.Height, FrameRate: p.FrameRate, Encoding: "jpeg", PTZ: true}
	}

	ch := make(chan Frame, 2)
	cam.SetFrameCallback(func(jpeg []byte, ts time.Time, seq uint64, _ string) {
		// Non-blocking: drop a frame rather than stall capture on a slow consumer.
		frame := make([]byte, len(jpeg))
		copy(frame, jpeg)
		select {
		case ch <- Frame{JPEG: frame, Seq: seq, TS: ts}:
		default:
		}
	})
	return ch, nil
}

func (s *v4l2Source) Close() error {
	if s.cam != nil {
		return s.cam.Close(context.Background())
	}
	return nil
}
