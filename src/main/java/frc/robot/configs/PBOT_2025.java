package frc.robot.configs;

import frc.robot.Robot.Config;

public class PBOT_2025 extends Config {
    public PBOT_2025() {
        swerve =
                swerve.configDriveGains(0.0, 0.0, 0.0)
                        .configEncoderOffsets(
                                0.27392578125, 0.237548828125, -0.123779296875, 0.44580078125);

        vision = vision.configCenterCameraAttached(true);

        this.algaeWheel.setAttached(false);
        this.elevator.setAttached(true);
        this.jaws.setAttached(false);
        this.lights.setAttached(false);
        this.tusks.setAttached(true);

        this.vision.setAttached(true);
        this.swerve.setAttached(true);

        this.driver.setAttached(true);
        this.codriver.setAttached(true);
    }
}
