package frc.robot.algaeWheel;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class AlgaeWheelStates {

    private static AlgaeWheel algaeWheel = Robot.getAlgaeWheel();

    public static void setupDefaultCommand() {
        algaeWheel.setDefaultCommand(algaeWheel.zero());
    }

    public static void setupBindings() {
        GrabAlgae.whileTrue(algaeWheel.grab());
        HoldAlgae.whileTrue(algaeWheel.hold());
        ReleaseAlgae.whileTrue(algaeWheel.release());
    }
}
