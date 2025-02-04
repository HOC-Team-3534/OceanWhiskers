package frc.robot.algaeWheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeWheel extends SubsystemBase {
    public static class AlgaeWheelConfig {}

    private AlgaeWheelConfig config;

    public AlgaeWheel(AlgaeWheelConfig config) {
        super();
        this.config = config;
    }
}
