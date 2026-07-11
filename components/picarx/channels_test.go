package picarx

import (
	"strings"
	"testing"

	"github.com/emergingrobotics/gorai/pkg/mesh"
)

// The gorai-mcp bridge builds its capability surface from gorai-channels, resolving
// each command's argument schema via the channel's Schema ref (DESIGN section 9 /
// R-150). Registering schemas alone leaves channels_on_mesh == 0 and the robot
// invisible, so every schema MUST have a matching channel that points back at it.
func TestChannelCatalogMirrorsSchemas(t *testing.T) {
	schemas := allSchemas()
	channels := allChannels()

	if len(channels) != len(schemas) {
		t.Fatalf("channel count %d != schema count %d", len(channels), len(schemas))
	}

	keyToSchema := map[string]mesh.SchemaDescriptor{}
	for _, s := range schemas {
		keyToSchema[s.SchemaKey()] = s
	}

	for _, ch := range channels {
		s, ok := keyToSchema[ch.Schema]
		if !ok {
			t.Errorf("channel %s references unknown schema key %q", ch.Subject, ch.Schema)
			continue
		}
		if ch.Subject != s.Name {
			t.Errorf("channel subject %q != schema name %q", ch.Subject, s.Name)
		}
		if ch.RobotID == "" {
			t.Errorf("channel %s has empty RobotID", ch.Subject)
		}
	}
}

// Direction reflects how the node actually serves each subject: commands and state
// are request/reply, streamed data is publish-only.
func TestChannelDirectionByType(t *testing.T) {
	want := map[string]mesh.Direction{
		"command": mesh.DirectionReqRep,
		"data":    mesh.DirectionPub,
		"state":   mesh.DirectionReqRep,
	}
	for _, ch := range allChannels() {
		typ := ch.Subject[strings.LastIndex(ch.Subject, ".")+1:]
		w, ok := want[typ]
		if !ok {
			t.Errorf("channel %s has unexpected type %q", ch.Subject, typ)
			continue
		}
		if ch.Direction != w {
			t.Errorf("channel %s (%s) direction = %q, want %q", ch.Subject, typ, ch.Direction, w)
		}
	}
}
