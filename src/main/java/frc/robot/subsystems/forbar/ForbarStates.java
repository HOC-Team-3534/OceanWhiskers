package frc.robot.subsystems.forbar;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class ForbarStates {

    private static Forbar forbar = Robot.getForbar();

    public static void setupDefaultCommand() {
        forbar.setDefaultCommand(forbar.zero());
    }

    public static void setupBindings() {
        GoToL2Coral.or(GoToL3Coral, GoToL4Coral)
                .and(() -> Robot.getElevator().getState().isNearTargetHeight())
                .onTrue(forbar.out());
        GoToL2Coral.or(GoToL3Coral, GoToL4Coral)
                .not()
                .and(() -> !forbar.getState().isIn())
                .onTrue(forbar.in());
    }
}
