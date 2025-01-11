// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CharacterizeDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller1 = new CommandXboxController(0);

    public final SwerveDriveSubsystem swerveDrive = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerveDrive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerveDrive.applyRequest(() -> drive.withVelocityX(-controller1.getLeftY() * MaxSpeed) // Drive forward
                                                                                                       // with negative
                                                                                                       // Y (forward)
                        .withVelocityY(-controller1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
                ));

        controller1.a().whileTrue(swerveDrive.applyRequest(() -> brake));
        controller1.b().whileTrue(swerveDrive.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        controller1.povDown().whileTrue(new CharacterizeDrive(swerveDrive, 1.0, 4.0));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldCentric()));

        swerveDrive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
