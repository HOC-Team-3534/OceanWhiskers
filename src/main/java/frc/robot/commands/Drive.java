package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.driver.Driver;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConfig;

public class Drive extends Command {
    private SwerveConfig defaultSwerve = new SwerveConfig();
    private double MaxSpeed =
            defaultSwerve
                    .getKSpeedAt12Volts()
                    .in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private Distance DriveCircumference =
            Meters.of(
                            new Translation2d(
                                            defaultSwerve.getKFrontLeftXPos(),
                                            defaultSwerve.getKFrontLeftYPos())
                                    .getNorm())
                    .times(2 * Math.PI);
    private double MaxAngularRate =
            RotationsPerSecond.of(
                            defaultSwerve
                                    .getKSpeedAt12Volts()
                                    .times(Seconds.of(1))
                                    .div(DriveCircumference)
                                    .magnitude())
                    .in(RadiansPerSecond); // 3/4 of a rotation per second

    private SwerveRequest idle = new SwerveRequest.Idle();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors

    static final double DEADBAND = 0.20;

    private final Swerve swerveDrive;
    private final Driver driver = Robot.getDriver();

    public Drive(Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        if (driver.getDriveFwdPositive()
                        + driver.getDriveLeftPositive()
                        + driver.getDriveCCWPositive()
                < 0.05) {
            swerveDrive.setControl(idle);
        } else {
            swerveDrive.setControl(
                    drive.withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                            .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                            .withRotationalRate(driver.getDriveCCWPositive() * MaxAngularRate));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(idle);
    }
}
