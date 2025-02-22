package frc.robot.configs;

import frc.robot.Robot.Config;

public class CBOT_2025 extends Config {

    public CBOT_2025() {
        super();
        swerve =
                // TODO: run system identification on swerve
                swerve.configDriveGains(0.077614, 2.3152, 0.30834)
                        .configEncoderOffsets(
                                0.38720703125, -0.35400390625, -0.314697265625, 0.2451171875);

        elevator.enableMotionMagic();

        tusks.testing();

        this.algaeWheel.setAttached(false);
        this.elevator.setAttached(true);
        this.jaws.setAttached(false);
        this.lights.setAttached(false);
        this.tusks.setAttached(true);

        this.vision.setAttached(false);
        this.swerve.setAttached(true);

        this.driver.setAttached(true);
        this.codriver.setAttached(true);
    }
}
