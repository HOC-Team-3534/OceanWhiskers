package frc.robot.tusks;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class TusksStates {
    private static Tusks tusks = Robot.getTusks();

    public static void setupDefaultCommand() {
        tusks.setDefaultCommand(
                Commands.either(
                        tusks.up(),
                        tusks.run(tusks::zero),
                        () -> Robot.getConfig().tusks.isMotionMagicEnabled()));
    }

    public static void setupBindings() {}
}
