package frc.robot.configs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import frc.robot.Robot.Config;

public class TBOT_2025 extends Config {
    public TBOT_2025() {
        super();
        swerve =
                swerve.configDriveGains(0.1566, 2.3096, 0.0232)
                        .configEncoderOffsets(
                                0.145751953125, -0.001953125, 0.326416015625, 0.22998046875);

        dtm.setOffsetFromWallToCenter(Inches.of(17.0));

        vision =
                vision.withFl_yaw(Degrees.of(45))
                        .withFr_yaw(Degrees.of(-45))
                        .withFrontY(Inches.of(20.5 / 2.0));

        this.algaeWheel.setAttached(false);
        this.elevator.setAttached(false);
        this.jaws.setAttached(false);
        this.lights.setAttached(false);
        this.tusks.setAttached(false);

        this.vision.setAttached(true);
        this.swerve.setAttached(true);

        this.driver.setAttached(true);
        this.codriver.setAttached(true);
    }
}
