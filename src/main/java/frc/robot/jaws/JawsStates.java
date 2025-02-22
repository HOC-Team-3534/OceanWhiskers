package frc.robot.jaws;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class JawsStates {

    private static Jaws jaws = Robot.getJaws();

    public static void setupDefaultCommand() {
        jaws.setDefaultCommand(jaws.zero());
    }

    public static void setupBindings() {
        RequestJawsClosed.and(JawsCanMove, JawsClosed.not()).onTrue(jaws.close());
        RequestJawsClosed.not().and(JawsCanMove, JawsOpened.not()).onTrue(jaws.open());
    }
}
