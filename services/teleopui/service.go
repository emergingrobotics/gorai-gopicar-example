package teleopui

import (
	"context"
	"embed"
	"encoding/json"
	"fmt"
	"io/fs"
	"log/slog"
	"net"
	"net/http"
	"os"
	"syscall"
	"time"

	"github.com/emergingrobotics/gorai/pkg/dashboard"
	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/nats-io/nats.go"
)

//go:embed web/*
var webFS embed.FS

func init() {
	registry.RegisterService("teleop-ui", "teleop-ui", New)
}

type Service struct {
	name      resource.Name
	nc        *nats.Conn
	log       *slog.Logger
	robotID   string
	listen    string
	cameraCap string
	hub       *dashboard.WebSocketHub
	srv       *http.Server
	ln        net.Listener
	cancel    context.CancelFunc
	subs      []*nats.Subscription
}

func New(ctx context.Context, deps registry.Dependencies, conf registry.Config) (any, error) {
	name, _ := conf["name"].(string)
	robotID, _ := conf["namespace"].(string)
	listen, _ := conf["listen"].(string)
	if listen == "" {
		listen = "0.0.0.0:8080"
	}
	camera, _ := conf["camera"].(string)
	if camera == "" {
		camera = "front"
	}
	v, err := deps.Get("nats")
	if err != nil {
		return nil, fmt.Errorf("nats dependency: %w", err)
	}
	nc, ok := v.(*nats.Conn)
	if !ok {
		return nil, fmt.Errorf("nats dependency is %T", v)
	}
	log := slog.Default()
	if lv, err := deps.Get("logger"); err == nil {
		if l, ok := lv.(*slog.Logger); ok {
			log = l
		}
	}
	return &Service{
		name:      resource.NewServiceName("gorai", "teleop-ui", name),
		nc:        nc,
		log:       log,
		robotID:   robotID,
		listen:    listen,
		cameraCap: camera,
	}, nil
}

func (s *Service) Name() resource.Name { return s.name }
func (s *Service) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}
func (s *Service) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *Service) addr() string {
	if s.ln != nil {
		return s.ln.Addr().String()
	}
	return s.listen
}

// handleControl decodes a browser control event and issues the mapped tool call.
// The browser reaches NATS only through here (C-001).
func (s *Service) handleControl(w http.ResponseWriter, r *http.Request) {
	var ev controlEvent
	if err := json.NewDecoder(r.Body).Decode(&ev); err != nil {
		http.Error(w, "bad event", http.StatusBadRequest)
		return
	}
	subject, args, ok := toolCall(ev, s.robotID)
	if !ok {
		http.Error(w, "unknown event", http.StatusBadRequest)
		return
	}
	payload, _ := json.Marshal(args)
	msg, err := s.nc.Request(subject, payload, time.Second)
	if err != nil {
		http.Error(w, err.Error(), http.StatusGatewayTimeout)
		return
	}
	w.Header().Set("Content-Type", "application/json")
	_, _ = w.Write(msg.Data)
}

// handleQuit lets the operator stop the whole robot from the GUI. It signals the
// process with SIGTERM so the normal graceful shutdown runs (motors stopped, camera
// released, NATS closed) rather than a hard exit.
func (s *Service) handleQuit(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	s.log.Info("shutdown requested from teleop-ui GUI")
	w.Header().Set("Content-Type", "application/json")
	_, _ = w.Write([]byte(`{"ok":true,"msg":"shutting down"}`))
	if f, ok := w.(http.Flusher); ok {
		f.Flush()
	}
	go func() {
		time.Sleep(150 * time.Millisecond) // let the response reach the browser first
		if p, err := os.FindProcess(os.Getpid()); err == nil {
			_ = p.Signal(syscall.SIGTERM)
		}
	}()
}

func (s *Service) Start(ctx context.Context) error {
	ctx, s.cancel = context.WithCancel(ctx)

	s.hub = dashboard.NewWebSocketHub()
	go s.hub.Run(ctx)
	s.startTelemetry(ctx)

	staticFS, err := fs.Sub(webFS, "web")
	if err != nil {
		return err
	}
	mux := http.NewServeMux()
	mux.Handle("/static/", http.StripPrefix("/static/", http.FileServer(http.FS(staticFS))))
	mux.HandleFunc("/stream/front", mjpegHandler(s.nc, fmt.Sprintf("gorai.%s.%s.data", s.robotID, s.cameraCap)))
	mux.HandleFunc("/ws", s.hub.HandleWebSocket)
	mux.HandleFunc("/control", s.handleControl)
	mux.HandleFunc("/quit", s.handleQuit)
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		b, _ := fs.ReadFile(staticFS, "index.html")
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write(b)
	})

	ln, err := net.Listen("tcp", s.listen)
	if err != nil {
		return fmt.Errorf("teleop-ui listen %s: %w", s.listen, err)
	}
	s.ln = ln
	s.srv = &http.Server{Handler: mux}
	go func() {
		if err := s.srv.Serve(ln); err != nil && err != http.ErrServerClosed {
			s.log.Error("teleop-ui server", "err", err)
		}
	}()
	s.log.Info("teleop-ui serving", "addr", s.addr())
	return nil
}

// startTelemetry subscribes to sensor data streams and pushes them to browsers.
func (s *Service) startTelemetry(ctx context.Context) {
	for _, cap := range []string{"battery", "distance", "grayscale", "line", "cliff"} {
		cap := cap
		subject := fmt.Sprintf("gorai.%s.%s.data", s.robotID, cap)
		sub, err := s.nc.Subscribe(subject, func(m *nats.Msg) {
			var payload map[string]any
			if err := json.Unmarshal(m.Data, &payload); err != nil {
				return
			}
			payload["cap"] = cap
			s.hub.BroadcastJSON(payload)
		})
		if err == nil {
			s.subs = append(s.subs, sub)
		}
	}
	// sysinfo has no data stream; poll its state once a second for the footer.
	go func() {
		t := time.NewTicker(time.Second)
		defer t.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-t.C:
				msg, err := s.nc.Request(fmt.Sprintf("gorai.%s.sysinfo.state", s.robotID), nil, time.Second)
				if err != nil {
					continue
				}
				var payload map[string]any
				if json.Unmarshal(msg.Data, &payload) == nil {
					payload["cap"] = "sysinfo"
					s.hub.BroadcastJSON(payload)
				}
			}
		}
	}()
}

func (s *Service) Close(ctx context.Context) error {
	if s.cancel != nil {
		s.cancel()
	}
	for _, sub := range s.subs {
		_ = sub.Unsubscribe()
	}
	if s.srv != nil {
		// The MJPEG stream and WebSocket are long-lived connections that never go
		// idle, so a plain Shutdown() blocks for as long as a browser is open. That
		// would wedge the whole robot's shutdown (Ctrl-C appears dead) and leave the
		// motors running. Bound it, then force-close any streaming connections.
		shutCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		if err := s.srv.Shutdown(shutCtx); err != nil {
			return s.srv.Close()
		}
	}
	return nil
}

var _ resource.Resource = (*Service)(nil)
