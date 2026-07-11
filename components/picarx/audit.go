package picarx

import (
	"fmt"

	"github.com/nats-io/nats.go"
)

// auditSubjects lists exactly what the audit stream captures. Raw video
// (front.data) is deliberately excluded (C-005, R-155): we keep every command
// and scalar reading, not every frame.
//
// Commands are audited via the dedicated gorai.<robot>.audit.command record
// (published by serveCommand), NOT the live *.command subject, and *.state is not
// captured either. Both are core NATS request/reply subjects: a JetStream-covered
// request subject makes JetStream ACK the caller's reply inbox and race the real
// handler's response, so the caller sees a PubAck instead of the tool reply.
func auditSubjects(robotID string) []string {
	return []string{
		fmt.Sprintf("gorai.%s.audit.command", robotID),
		fmt.Sprintf("gorai.%s.*.event", robotID),
		fmt.Sprintf("gorai.%s.battery.data", robotID),
		fmt.Sprintf("gorai.%s.distance.data", robotID),
		fmt.Sprintf("gorai.%s.grayscale.data", robotID),
		fmt.Sprintf("gorai.%s.line.data", robotID),
		fmt.Sprintf("gorai.%s.cliff.data", robotID),
	}
}

func ensureAuditStream(nc *nats.Conn, robotID string) error {
	js, err := nc.JetStream()
	if err != nil {
		return err
	}
	cfg := &nats.StreamConfig{
		Name:      "picarx-audit",
		Subjects:  auditSubjects(robotID),
		Retention: nats.LimitsPolicy,
		Storage:   nats.FileStorage,
	}
	if _, err := js.AddStream(cfg); err != nil {
		// AddStream errors if it already exists with a different config; update it.
		if _, uerr := js.UpdateStream(cfg); uerr != nil {
			return fmt.Errorf("audit stream: add=%w update=%v", err, uerr)
		}
	}
	return nil
}
