package frc.robot.configs;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.hocLib.mechanism.TalonSRXArm.ArmSlotConfig;
import frc.robot.Robot.Config;

public class PBOT_2025 extends Config {
    public PBOT_2025() {
        super();
        swerve =
                // TODO: run system identification on swerve
                swerve.configDriveGains(0.17223, 2.3111, 0.12424)
                        .configEncoderOffsets(
                                0.27392578125, 0.237548828125, -0.123779296875, 0.44580078125);

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = 3.0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        // spotless:off
        //https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=83&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A18%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A3.66%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A54%2C%22u%22%3A%22in%22%7D
        //spotless:on

        slot0Configs.kG = 0.805;
        slot0Configs.kS = 0.0559;
        slot0Configs.kV = 0.11793;
        slot0Configs.kA = 0.0048; // recalc says 0.007 V * s^2 / rot

        elevator.setSlot0Configs(slot0Configs);

        elevator.configMaxLinearPosition(Inches.of(54.625));
        elevator.configMaxPosition(Rotations.of(23.713));

        elevator.enableMotionMagic();

        tusks.setSlotConfigs(
                new ArmSlotConfig(0.6425, 0.14478, 0.5, 0, 0, 0.87),
                new ArmSlotConfig(0.365, 0.705, 0.5, 0, 0, 0.87));

        tusks.enableMotionMagic();

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
