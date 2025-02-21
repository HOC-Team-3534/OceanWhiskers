package frc.robot.algaeWheel;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class AlgaeWheelStates {

    private static AlgaeWheel algaeWheel = Robot.getAlgaeWheel();

    public static void setupDefaultCommand() {
        algaeWheel.setDefaultCommand(algaeWheel.zero());
    }

    public static void setupBindings() {
        RequestGrabAlgae.and(JawsRelated.Closed).whileTrue(algaeWheel.grab());
        HoldAlgae.and(JawsRelated.Closed).whileTrue(algaeWheel.hold());
        RequestReleaseAlgae.and(JawsRelated.Closed).whileTrue(algaeWheel.release());
    }
}
