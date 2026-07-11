# NATS Authorization for the LAN-Exposed Bus (R-141, D-3)

**Decision (D-3):** the LAN-exposed bus uses **NKey/JWT accounts**; the credential set is
the robot's authorization boundary and maps to the Composite Robot model. A relaxed no-auth
mode is bench-only and MUST NOT be the deployed default.

## Verified constraint (gorai core, 2026-07)

Read against `/gorai-all/gorai`:

- `config.NATSConfig` (RDL `nats`) exposes `url`, `urls`, `jetstream`, `credentials_file`,
  `tls`, `external`, timeouts — a **client** `credentials_file`, but no server-side account
  model.
- `embeddednats.Config` is `{Host, Port, JetStream, JetStreamDir, TLS, Logger}` — **no
  accounts / operator / authorization / resolver**. The embedded server therefore cannot
  enforce NKey/JWT accounts; it accepts any client that can reach the port.

**Consequence:** with the embedded server, binding to `0.0.0.0` (R-140) exposes an
*unauthenticated* bus on the LAN. That satisfies R-140 (reachability) but NOT R-141
(authorization). These two requirements cannot both be met by the embedded server as it
stands.

## Path to satisfy R-141

### Option A (deployable today) — external NATS with accounts
Run a standalone `nats-server` configured with an operator, a `picarx` account, and users,
and point the robot at it:

- `robot.json` -> `"nats": { "external": true, "url": "nats://<host>:4222", "credentials_file": "picarx-robot.creds", "jetstream": true }`
- The robot's own components/services connect with `picarx-robot.creds`.
- External agents get their own user creds in the same `picarx` account (or a peer account
  with import/export grants). Revoking a cred removes a participant — the credential set is
  the boundary.
- Generate with `nsc`: operator -> account `picarx` -> users `robot` and `agent`; export the
  JWT resolver or use a memory resolver in the server config.

This meets R-141 without core changes, at the cost of running one extra process on the Pi
(or a nearby host).

### Option B (preferred long-term) — extend gorai core
Add authorization to `embeddednats.Config` (accounts/users or an operator+resolver) and
surface it in RDL `NATSConfig` (e.g. an `auth`/`accounts` block). Then the single embedded
server both binds the LAN and enforces accounts, keeping the one-binary promise. **This is a
gorai-core enhancement to raise upstream; it is a prerequisite for R-141 under embedded
NATS.**

## Bench mode (non-default)
For hardware bring-up on a trusted, isolated LAN, the embedded server with no auth
(current default) is acceptable **temporarily**. The deployed configuration MUST use
Option A (or Option B once available). Do not ship the unauthenticated embedded bus.

## Status
- R-140 (LAN bind): met via `nats.url = nats://0.0.0.0:4222`.
- R-141 (authorization): **NOT met by the embedded server** (core gap above). Deploy via
  Option A; track Option B upstream. Recorded here rather than silently skipped.
