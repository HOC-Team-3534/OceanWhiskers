// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.CharacterizeDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArcSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TusksSubsystem;

public class RobotContainer {
    private static final CommandXboxController controller1 = new CommandXboxController(0);
    private static final CommandXboxController controller2 = new CommandXboxController(1);

    private static final SwerveDriveSubsystem swerveDrive = TunerConstants.createDrivetrain();

    private static final boolean ELEVATOR_ENABLED = false;
    private static final boolean ARC_ENABLED = false;

    private static final boolean TUSKS_ENABLED = false;

    private static final Optional<ElevatorSubsystem> elevator = ELEVATOR_ENABLED ? Optional.of(new ElevatorSubsystem())
            : Optional.empty();
    private static final Optional<ArcSubsystem> arc = ARC_ENABLED ? Optional.of(new ArcSubsystem()) : Optional.empty();
    @SuppressWarnings("unused")
    private static final Optional<TusksSubsystem> tusks = TUSKS_ENABLED ? Optional.of(new TusksSubsystem())
            : Optional.empty();
    private static final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    private final ShuffleboardTab configsTab = Shuffleboard.getTab("Configs");

    public RobotContainer() {
        configureBindings();

        autonChooser.setDefaultOption("No Auton", Commands.none());
        autonChooser.addOption("Drive Forward", Autos.driveForward(Feet.of(2)));

        SmartDashboard.putData(autonChooser);

    }

    private void configureBindings() {
        elevator.ifPresent(e -> {
            controller2.a().whileTrue(e.l1());
            controller2.b().whileTrue(e.l2());
            controller2.x().whileTrue(e.l3());
            controller2.y().whileTrue(e.l4());
        });

        arc.ifPresent(a -> {
            controller1.rightTrigger(0.25).whileTrue(a.intake());
            controller1.leftTrigger(0.25).whileTrue(a.extake());
        });

        tusks.ifPresent(t -> {
            controller2.leftStick().onTrue(t.deploy());
        });

        controller1.a().whileTrue(Commands.deferredProxy(() -> Autos.dtmToReef()));
        controller1.b().whileTrue(Commands.deferredProxy(() -> Autos.dtmToHumanPlayerStation()));

        controller1.povDown()
                .whileTrue(new CharacterizeDrive(swerveDrive, Volts.per(Second).ofNative(1), Seconds.of(4.0)));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDrive;
    }

    public static Optional<ElevatorSubsystem> getElevatorSubsystem() {
        return elevator;
    }

    public static CommandXboxController getController1() {
        return controller1;
    }

    public static void photonVisionPeriodic() {
        photonVision.periodic();
    }
}
