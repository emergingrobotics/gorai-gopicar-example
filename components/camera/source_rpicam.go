//go:build rpicam

// Package camera's capture source for Raspberry Pi CSI cameras (e.g. the OV5647
// PiCam). On a Pi 5, /dev/video0 is the raw CFE (rp1-cfe) and cannot be driven
// as a plain V4L2 device -- VIDIOC_STREAMON returns EINVAL because capture must
// go through the libcamera media pipeline (CFE -> ISP debayer). This source
// shells out to rpicam-vid, which orchestrates that pipeline and emits an MJPEG
// stream on stdout; we split it into individual JPEG frames.
//
// Built only with -tags rpicam (on the Pi). Mutually exclusive with the v4l2
// source, which targets USB/UVC webcams.
package camera

import (
	"bufio"
	"context"
	"fmt"
	"os/exec"
	"strconv"
	"time"

	"github.com/emergingrobotics/gorai/pkg/registry"
)

func init() {
	sourceFactory = func(conf registry.Config) (Source, error) {
		return &rpicamSource{conf: conf}, nil
	}
}

type rpicamSource struct {
	conf   registry.Config
	cmd    *exec.Cmd
	cancel context.CancelFunc
	props  Properties
}

func (s *rpicamSource) Properties() Properties { return s.props }

func (s *rpicamSource) Start(ctx context.Context) (<-chan Frame, error) {
	geti := func(key string, def int) int {
		if v, ok := s.conf[key].(float64); ok {
			return int(v)
		}
		return def
	}
	width := geti("width", 640)
	height := geti("height", 480)
	fps := geti("fps", 15)
	quality := geti("jpeg_quality", 70)

	ctx, s.cancel = context.WithCancel(ctx)

	// -t 0: run until killed; -n: no preview; --flush: emit each frame ASAP.
	args := []string{
		"--codec", "mjpeg",
		"-o", "-",
		"-t", "0",
		"-n",
		"--flush",
		"--width", strconv.Itoa(width),
		"--height", strconv.Itoa(height),
		"--framerate", strconv.Itoa(fps),
		"-q", strconv.Itoa(quality),
	}
	cmd := exec.CommandContext(ctx, "rpicam-vid", args...)
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		s.cancel()
		return nil, fmt.Errorf("rpicam-vid stdout pipe: %w", err)
	}
	// rpicam-vid is chatty on stderr; discard it (goes to the null device).
	cmd.Stderr = nil
	if err := cmd.Start(); err != nil {
		s.cancel()
		return nil, fmt.Errorf("start rpicam-vid (is it installed? is the camera enabled?): %w", err)
	}
	s.cmd = cmd
	s.props = Properties{Width: width, Height: height, FrameRate: float64(fps), Encoding: "jpeg", PTZ: false}

	ch := make(chan Frame, 2)
	go func() {
		defer close(ch)
		defer s.cancel()
		splitMJPEG(ctx, bufio.NewReaderSize(stdout, 1<<20), func(jpeg []byte, seq uint64) {
			// Non-blocking: drop a frame rather than stall capture on a slow consumer.
			select {
			case ch <- Frame{JPEG: jpeg, Seq: seq, TS: time.Now()}:
			case <-ctx.Done():
			default:
			}
		})
		_ = cmd.Wait()
	}()
	return ch, nil
}

func (s *rpicamSource) Close() error {
	if s.cancel != nil {
		s.cancel()
	}
	if s.cmd != nil && s.cmd.Process != nil {
		_ = s.cmd.Process.Kill()
	}
	return nil
}

// splitMJPEG reads a concatenated-JPEG (MJPEG) byte stream and calls emit once
// per complete frame, delimited by the JPEG SOI (0xFFD8) and EOI (0xFFD9)
// markers. A literal 0xFFD9 never occurs inside JPEG entropy-coded data (0xFF is
// byte-stuffed as 0xFF00 there), so the EOI scan is unambiguous.
func splitMJPEG(ctx context.Context, r *bufio.Reader, emit func(jpeg []byte, seq uint64)) {
	var buf []byte
	var seq uint64
	inFrame := false
	for {
		if ctx.Err() != nil {
			return
		}
		b, err := r.ReadByte()
		if err != nil {
			return
		}
		if !inFrame {
			// Hunt for SOI (0xFF 0xD8). Keep at most a trailing 0xFF as context.
			if len(buf) == 1 && buf[0] == 0xFF && b == 0xD8 {
				buf = []byte{0xFF, 0xD8}
				inFrame = true
			} else if b == 0xFF {
				buf = []byte{0xFF}
			} else {
				buf = buf[:0]
			}
			continue
		}
		buf = append(buf, b)
		if n := len(buf); n >= 2 && buf[n-2] == 0xFF && b == 0xD9 {
			frame := make([]byte, n)
			copy(frame, buf)
			seq++
			emit(frame, seq)
			buf = buf[:0]
			inFrame = false
		}
	}
}
