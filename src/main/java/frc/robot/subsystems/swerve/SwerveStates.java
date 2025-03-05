package frc.robot.subsystems.swerve;

import static frc.robot.RobotStates.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.controllers.Driver;

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

        var fwd = new ChassisSpeeds(0.7, 0.0, 0.0);
        var back = new ChassisSpeeds(-0.7, 0.0, 0.0);
        var left = new ChassisSpeeds(0.0, 0.3, 0.0);
        var right = new ChassisSpeeds(0.0, -0.3, 0.0);

        RobotCentricForward.whileTrue(swerve.run(() -> swerve.driveWithSpeeds(fwd)));
        RobotCentricBackward.whileTrue(swerve.run(() -> swerve.driveWithSpeeds(back)));
        RobotCentricLeft.whileTrue(swerve.run(() -> swerve.driveWithSpeeds(left)));
        RobotCentricRight.whileTrue(swerve.run(() -> swerve.driveWithSpeeds(right)));
    }
}
