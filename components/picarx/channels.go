package picarx

import (
	"context"
	"strings"

	"github.com/emergingrobotics/gorai/pkg/mesh"
)

// channelDirs maps a subject's trailing type token to how the node serves it:
// commands and state snapshots are request/reply, streamed sensor data is publish.
var channelDirs = map[string]struct {
	dir mesh.Direction
	qos mesh.QoS
}{
	"command": {mesh.DirectionReqRep, mesh.QoSReliable},
	"state":   {mesh.DirectionReqRep, mesh.QoSReliable},
	"data":    {mesh.DirectionPub, mesh.QoSBestEffort},
}

// allChannels derives the channel catalog from allSchemas so the two never drift:
// gorai-channels is the authoritative discovery surface for the gorai-mcp bridge
// (DESIGN section 9), and each channel carries the schema key so the bridge can
// resolve a command's argument schema. Schema names are already subject-shaped
// (gorai.<robot>.<cap>.<type>), so they serve directly as channel subjects.
func allChannels() []mesh.ChannelDescriptor {
	schemas := allSchemas()
	chans := make([]mesh.ChannelDescriptor, 0, len(schemas))
	for _, s := range schemas {
		robot, typ := robotAndType(s.Name)
		dq := channelDirs[typ]
		chans = append(chans, mesh.ChannelDescriptor{
			Subject:     s.Name,
			Schema:      s.SchemaKey(),
			RobotID:     robot,
			Direction:   dq.dir,
			QoS:         dq.qos,
			Description: s.Description,
		})
	}
	return chans
}

// robotAndType splits the robot ID and trailing type token out of a subject-shaped
// name (gorai.<robot>.<cap>.<type>); it returns empty strings for malformed names.
func robotAndType(name string) (robot, typ string) {
	parts := strings.Split(name, ".")
	if len(parts) < 4 || parts[0] != "gorai" {
		return "", ""
	}
	return parts[1], parts[len(parts)-1]
}

func (c *Component) registerChannels(ctx context.Context) error {
	client, err := mesh.NewClient(c.nc)
	if err != nil {
		return err
	}
	for _, ch := range allChannels() {
		if err := client.RegisterChannel(ctx, ch); err != nil {
			c.log.Warn("channel register failed", "subject", ch.Subject, "err", err)
		}
	}
	return nil
}
