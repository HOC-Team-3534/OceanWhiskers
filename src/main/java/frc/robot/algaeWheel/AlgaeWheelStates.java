package frc.robot.algaeWheel;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class AlgaeWheelStates {

    private static AlgaeWheel algaeWheel = Robot.getAlgaeWheel();

    public static void setupDefaultCommand() {
        algaeWheel.setDefaultCommand(algaeWheel.zero());
    }

    public static void setupBindings() {
        RequestGrabAlgae.and(JawsClosed).whileTrue(algaeWheel.grab());
        HoldAlgae.and(JawsClosed).whileTrue(algaeWheel.hold());
        RequestReleaseAlgae.and(JawsClosed).whileTrue(algaeWheel.release());
    }
}
