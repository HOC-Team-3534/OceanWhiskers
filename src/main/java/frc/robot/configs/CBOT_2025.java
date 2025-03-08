package frc.robot.configs;

import frc.robot.Robot.Config;

public class CBOT_2025 extends Config {

    public CBOT_2025() {
        super();
        swerve =
                swerve.configDriveGains(0.077614, 2.3152, 0.30834)
                        .configEncoderOffsets(
                                0.376708984375, -0.3603515625, 0.325927734375, 0.1435546875);

        elevator.enableMotionMagic();

        this.forbar.setAttached(false);
        this.elevator.setAttached(true);
        this.jaws.setAttached(true);
        this.lights.setAttached(true);

        this.vision.setAttached(true);
        this.swerve.setAttached(true);

        this.driver.setAttached(true);
        this.codriver.setAttached(true);
    }
}
