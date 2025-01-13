// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.CharacterizeDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    private static final CommandXboxController controller1 = new CommandXboxController(0);

    private static final SwerveDriveSubsystem swerveDrive = TunerConstants.createDrivetrain();
    @SuppressWarnings("unused")
    private static final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    private final ShuffleboardTab configsTab = Shuffleboard.getTab("Configs");

    public RobotContainer() {
        configureBindings();

        autonChooser.setDefaultOption("No Auton", Commands.none());
        autonChooser.addOption("Drive Forward", Autos.driveForward(Feet.of(3)));

        configsTab.add(autonChooser);
    }

    private void configureBindings() {
        controller1.povDown().whileTrue(new CharacterizeDrive(swerveDrive, 1.0, 4.0));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDrive;
    }

    public static CommandXboxController getController1() {
        return controller1;
    }
}
