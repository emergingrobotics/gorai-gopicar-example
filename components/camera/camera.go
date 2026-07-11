package camera

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"

	"github.com/emergingrobotics/gorai/pkg/registry"
	"github.com/emergingrobotics/gorai/pkg/resource"
	"github.com/emergingrobotics/gorai/pkg/subjects"
	"github.com/nats-io/nats.go"
)

func init() {
	registry.RegisterComponent("camera", "picam", New)
}

// sourceFactory builds the capture source. Overridden by the v4l2 build (Task 10);
// the default returns an error so a hostless build fails loudly rather than silently.
var sourceFactory = func(conf registry.Config) (Source, error) {
	return nil, fmt.Errorf("no camera source compiled in; build with -tags v4l2 on the Pi")
}

type Component struct {
	name    resource.Name
	nc      *nats.Conn
	log     *slog.Logger
	subj    *subjects.Builder
	capName string
	src     Source
	cancel  context.CancelFunc
	subs    []*nats.Subscription
}

func New(ctx context.Context, deps registry.Dependencies, conf registry.Config) (any, error) {
	name, _ := conf["name"].(string)
	robotID, _ := conf["namespace"].(string)

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
	src, err := sourceFactory(conf)
	if err != nil {
		return nil, err
	}
	c := &Component{
		name:    resource.NewComponentName("gorai", "camera", name),
		nc:      nc,
		log:     log,
		subj:    subjects.NewBuilder(robotID),
		capName: name,
		src:     src,
	}
	return c, nil
}

func (c *Component) Name() resource.Name { return c.name }
func (c *Component) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}
func (c *Component) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (c *Component) Start(ctx context.Context) error {
	ctx, c.cancel = context.WithCancel(ctx)
	frames, err := c.src.Start(ctx)
	if err != nil {
		return fmt.Errorf("camera source start: %w", err)
	}
	dataSubj := c.subj.ComponentData(c.capName)

	// state reply: resolution/encoding/fps/ptz
	sub, err := c.nc.Subscribe(c.subj.ComponentState(c.capName), func(m *nats.Msg) {
		p := c.src.Properties()
		b, _ := json.Marshal(map[string]any{
			"w": p.Width, "h": p.Height, "enc": p.Encoding, "fps": p.FrameRate, "ptz": p.PTZ,
		})
		_ = m.Respond(b)
	})
	if err == nil {
		c.subs = append(c.subs, sub)
	}

	// single capture, published to NATS. I-005/C-006.
	go func() {
		for {
			select {
			case <-ctx.Done():
				return
			case f, ok := <-frames:
				if !ok {
					return
				}
				_ = c.nc.Publish(dataSubj, f.JPEG)
			}
		}
	}()
	return nil
}

func (c *Component) Close(context.Context) error {
	if c.cancel != nil {
		c.cancel()
	}
	for _, s := range c.subs {
		_ = s.Unsubscribe()
	}
	return c.src.Close()
}

var _ resource.Resource = (*Component)(nil)
