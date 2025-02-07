package frc.robot.swerve;

import frc.robot.Robot;
import frc.robot.driver.Driver;

public class SwerveStates {
    private static Swerve swerve = Robot.getSwerve();
    private static Driver driver = Robot.getDriver();

    public static void setupDefaultCommand() {
        swerve.setDefaultCommand(
                swerve.drive(
                        driver::getDriveFwdPositive,
                        driver::getDriveLeftPositive,
                        driver::getDriveCCWPositive));
    }

    public static void setupBindings() {}
}
