// Command gorai-picarx is a teleoperable PiCar-X GoRAI robot: it registers the
// picarx and camera components and the teleop-ui service, then runs the robot.
package main

import (
	gorai "github.com/emergingrobotics/gorai/pkg/gorai"

	// Blank imports are the component/service manifest; each self-registers
	// via init() -> registry.RegisterComponent / RegisterService.
	_ "github.com/emergingrobotics/gorai-picarx/components/camera"
	_ "github.com/emergingrobotics/gorai-picarx/components/picarx"
	_ "github.com/emergingrobotics/gorai-picarx/services/teleopui"
)

func main() {
	gorai.Run()
}
