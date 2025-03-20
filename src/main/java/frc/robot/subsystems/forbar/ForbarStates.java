package frc.robot.subsystems.forbar;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.RobotStates.*;

import frc.robot.Robot;
import frc.robot.controllers.Codriver;

public class ForbarStates {

    private static Forbar forbar = Robot.getForbar();
    private static Codriver codriver = Robot.getCodriver();

    public static void setupDefaultCommand() {
        forbar.setDefaultCommand(forbar.zeroOrHold());
    }

    public static void setupBindings() {
        GoToL2Coral.or(GoToL3Coral, GoToL4Coral)
                .and(codriver.ForceForbarIn.not(),
                        () -> Robot.getElevator().getState().isNearTargetHeight(),
                        () -> Robot.getElevator().getTargetHeight().gt(Inches.of(0)))
                .onTrue(forbar.out());
        GoToL2Coral.or(GoToL3Coral, GoToL4Coral).not().or(codriver.ForceForbarIn).onTrue(forbar.in());
    }
}
