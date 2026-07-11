package teleopui

import (
	"fmt"
	"net/http"

	"github.com/nats-io/nats.go"
)

// mjpegHandler bridges the NATS JPEG frame stream to an HTTP MJPEG response.
// Modeled on gorai dashboard/cameras.StreamHandler but over the raw *nats.Conn
// (services do not get the *gorainats.Client wrapper). DESIGN 11 R9, R-134.
func mjpegHandler(nc *nats.Conn, subject string) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		flusher, ok := w.(http.Flusher)
		if !ok {
			http.Error(w, "streaming unsupported", http.StatusInternalServerError)
			return
		}
		w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
		w.Header().Set("Cache-Control", "no-cache")
		w.Header().Set("Connection", "close")
		// Flush the status line + headers now so the client starts rendering
		// before the first frame arrives (MJPEG clients expect this).
		w.WriteHeader(http.StatusOK)
		flusher.Flush()

		frameCh := make(chan []byte, 2)
		sub, err := nc.Subscribe(subject, func(m *nats.Msg) {
			select {
			case frameCh <- m.Data:
			default: // drop when the client is slow
			}
		})
		if err != nil {
			http.Error(w, "subscribe failed", http.StatusServiceUnavailable)
			return
		}
		defer sub.Unsubscribe()

		ctx := r.Context()
		for {
			select {
			case <-ctx.Done():
				return
			case frame := <-frameCh:
				if _, err := fmt.Fprintf(w, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(frame)); err != nil {
					return
				}
				if _, err := w.Write(frame); err != nil {
					return
				}
				if _, err := fmt.Fprint(w, "\r\n"); err != nil {
					return
				}
				flusher.Flush()
			}
		}
	}
}
