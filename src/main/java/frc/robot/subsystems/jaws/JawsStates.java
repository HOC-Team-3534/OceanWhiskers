package frc.robot.subsystems.jaws;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class JawsStates {

    private static Jaws jaws = Robot.getJaws();

    public static void setupDefaultCommand() {
        jaws.setDefaultCommand(jaws.zero());
    }

    public static void setupBindings() {
        RequestJawsOut.and(JawsCanMove, JawsOut.not()).onTrue(jaws.out());
        RequestJawsOut.not().and(JawsCanMove, JawsIn.not()).onTrue(jaws.in());
    }
}
