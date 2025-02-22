package frc.robot.swerve;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

    public static void setupBindings() {
        SwerveQuasiasticForward.whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        SwerveQuasiasticBackward.whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        SwerveDynamicForward.whileTrue(swerve.sysIdDynamic(Direction.kForward));
        SwerveDynamicBackward.whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    }
}
