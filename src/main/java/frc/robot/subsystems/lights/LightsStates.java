package frc.robot.subsystems.lights;

import static frc.robot.RobotStates.*;

import frc.robot.Robot;

public class LightsStates {
    private static Lights lights = Robot.getLights();

    public static void setupDefaultCommand() {
        lights.setDefaultCommand(lights.normal());
    }

    public static void setupBindings() {
        DrivingAutonomously.whileTrue(lights.drivingAutonomously());
    }
}
