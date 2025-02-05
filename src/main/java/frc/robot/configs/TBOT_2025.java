package frc.robot.configs;

import frc.robot.Robot.Config;

public class TBOT_2025 extends Config {
    public TBOT_2025() {
        swerve =
                swerve.configDriveGains(0.1566, 2.3096, 0.0232)
                        .configEncoderOffsets(
                                0.145751953125, -0.001953125, 0.326416015625, 0.22998046875);
    }
}
