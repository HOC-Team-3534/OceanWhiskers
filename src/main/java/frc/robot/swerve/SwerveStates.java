package frc.robot.swerve;

import static frc.robot.RobotStates.SwerveRelated.*;

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
        QuasiasticForward.whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        QuasiasticBackward.whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        DynamicForward.whileTrue(swerve.sysIdDynamic(Direction.kForward));
        DynamicBackward.whileTrue(swerve.sysIdDynamic(Direction.kReverse));
    }
}
