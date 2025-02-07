package frc.robot.jaws;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class JawsStates {

    private static Jaws jaws = Robot.getJaws();

    public static void setupDefaultCommand() {
        jaws.setDefaultCommand(jaws.zero());
    }

    public static void setupBindings() {
        GrabAlgae.or(HoldAlgae, ReleaseAlgae).onTrue(jaws.close()).onFalse(jaws.open());
    }
}
