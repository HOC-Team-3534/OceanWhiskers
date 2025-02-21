package frc.robot.jaws;

import static frc.robot.RobotStates.*;
import static frc.robot.RobotStates.JawsRelated.*;

import frc.robot.Robot;

public class JawsStates {

    private static Jaws jaws = Robot.getJaws();

    public static void setupDefaultCommand() {
        jaws.setDefaultCommand(jaws.zero());
    }

    public static void setupBindings() {
        RequestJawsClosed.and(CanMove, Closed.not()).onTrue(jaws.close());
        RequestJawsClosed.not().and(CanMove, Opened.not()).onTrue(jaws.open());
    }
}
