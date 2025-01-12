// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CharacterizeDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController controller1 = new CommandXboxController(0);

    public static final SwerveDriveSubsystem swerveDrive = TunerConstants.createDrivetrain();
    public static final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        controller1.a().whileTrue(swerveDrive.applyRequest(() -> brake));
        controller1.b().whileTrue(swerveDrive.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        controller1.povDown().whileTrue(new CharacterizeDrive(swerveDrive, 1.0, 4.0));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public static SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDrive;
    }
}
