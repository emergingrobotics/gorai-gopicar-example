package camera

import "log/slog"

// rtspServer is a no-op placeholder. The real RTP/JPEG server (D-1, R-123) is
// implemented with github.com/bluenviron/gortsplib/v4, whose API is version-
// specific and must be wired verbatim against the installed source. That module
// is NOT available in the current build environment (the proxy serves an empty
// stub), so RTSP is not compiled here: the NATS frame stream and in-page MJPEG
// (the other two video paths) work; RTSP is completed where gortsplib is
// fetchable (the Pi / a networked host). See docs/BRINGUP.md and the plan Task 18.
type rtspServer struct{}

// newRTSPServer logs that RTSP is not built and returns a no-op server so the
// camera component still runs (publishing frames to NATS + MJPEG).
func newRTSPServer(listen, path string) (*rtspServer, error) {
	slog.Default().Warn("RTSP requested but not compiled in this build (gortsplib unavailable); NATS+MJPEG video still active",
		"listen", listen, "path", path)
	return &rtspServer{}, nil
}

func (r *rtspServer) push(Frame)   {}
func (r *rtspServer) close() error { return nil }
