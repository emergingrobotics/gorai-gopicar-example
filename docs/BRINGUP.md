# PiCar-X Bring-Up (on the Raspberry Pi)

Prereqs: 64-bit Pi OS, Wi-Fi up, camera enabled (`/dev/video0` present), gopicar
udev/permissions per gopicar docs, and a networked host that can fetch Go modules
(needed for the real `gortsplib` — see step 2 and the RTSP note).

1. Cross-compile + deploy from the dev host: `make deploy DEPLOY_HOST=pi@<host>`
   (copies the binary, `robot.json`, `calibration.json`). For the camera, build WITH the
   `v4l2` tag so the real capture source is linked:
   `go build -tags v4l2 -o bin/picarx .` (or wire the tag into your `gorai build`).
2. **RTSP (deferred):** the RTP/JPEG server uses `github.com/bluenviron/gortsplib/v4`,
   which was unavailable in the drafting environment (proxy served an empty stub), so
   `components/camera/rtsp.go` is a no-op today. On a host with real module access, add the
   dep and implement `newRTSPServer/push/close` against the installed gortsplib server +
   `format.MJPEG` example (plan Task 18), then rebuild. Until then, video is available
   in-page (MJPEG) and over NATS (`gorai.picarx.front.data`) — criteria 2/3/4-page still
   pass; only the external RTSP viewer (criterion 4-RTSP / B-007) waits on this.
3. Calibrate: center each servo, record raw angles, write `calibration.json`
   (see gopicar examples/picarctl `calibrate`).
4. For LAN reach (R-140), set `nats.url` in `robot.json` to the Pi's routable address before
   running, e.g. `"nats": { "url": "nats://raspberrypi.local:4222", "jetstream": true }`
   (the committed default `127.0.0.1` is local-only; `0.0.0.0` does NOT work — gorai dials
   the same URL locally and rejects `0.0.0.0`; see DESIGN §11). For LAN + auth, use an
   external nats-server per docs/NATS-AUTH.md.
5. `./bin/picarx` (or `gorai run robot.json`). One process, no others. [Criterion 1]
6. Browser on the same Wi-Fi -> `http://<pi>:8080/`:
   - live video panel renders. [Criteria 2, 3]
   - battery/distance/grayscale/line/cliff/system update with no refresh. [Criterion 2]
   - throttle/steer/cam sliders drive; W/A/S/D + arrows drive; camera pans/tilts. [Criterion 3]
   - release all controls / close tab / drop Wi-Fi -> car stops within ~0.5s;
     spacebar / STOP e-stops immediately. [Criterion 5]
7. From a second LAN machine:
   - `gorai mesh services` / `gorai mesh schemas` list picarx + camera. [Criterion 6]
   - `nats sub 'gorai.picarx.>'` shows command + telemetry (no video frames in
     the audit stream). [Criterion 7, C-005]
   - once RTSP is built: `vlc rtsp://<pi>:8554/front` renders live video while the page
     also streams. [Criterion 4, B-007]
8. Cliff test (carefully, wheels off the edge): lift the front over a table edge ->
   motors stop, `gorai.picarx.cliff.event` fires, drive is refused until cleared. [B-005]

## Security note (R-141)
The embedded NATS server binds `0.0.0.0` (R-140) but does not enforce accounts (gorai-core
gap; see `docs/NATS-AUTH.md`). For any non-isolated network, deploy with an external NATS
server configured with NKey/JWT accounts (Option A there). Do not expose the
unauthenticated embedded bus on an untrusted LAN.
