// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.CharacterizeDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.robot_specific.RobotConstants.EnabledSubsystems;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JawsSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TusksSubsystem;

public class RobotContainer {
    private static final CommandXboxController controller1 = new CommandXboxController(0);
    private static final CommandXboxController controller2 = new CommandXboxController(1);

    private static final SwerveDriveSubsystem swerveDrive = TunerConstants.createDrivetrain();

    private static final Optional<ElevatorSubsystem> elevator = EnabledSubsystems.ELEVATOR_ENABLED
            ? Optional.of(new ElevatorSubsystem())
            : Optional.empty();
    private static final Optional<JawsSubsystem> jaws = EnabledSubsystems.JAWS_ENABLED
            ? Optional.of(new JawsSubsystem())
            : Optional.empty();
    @SuppressWarnings("unused")
    private static final Optional<TusksSubsystem> tusks = EnabledSubsystems.TUSKS_ENABLED
            ? Optional.of(new TusksSubsystem())
            : Optional.empty();
    private static final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();
    private static final LightsSubsystem lights = new LightsSubsystem();

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private final Field2d goalPoseField = new Field2d();

    public RobotContainer() {
        configureBindings();

        autonChooser.setDefaultOption("No Auton", Commands.none());
        autonChooser.addOption("Drive Forward", Autos.driveForward(Feet.of(2)));

        SmartDashboard.putData(autonChooser);

        Shuffleboard.getTab("Testing").add(goalPoseField).withWidget(BuiltInWidgets.kField);
    }

    private void configureBindings() {
        elevator.ifPresent(e -> {
            controller2.a().whileTrue(e.l1());
            controller2.b().whileTrue(e.l2());
            controller2.x().whileTrue(e.l3());
            controller2.y().whileTrue(e.l4());
        });

        jaws.ifPresent(a -> {
            controller1.rightTrigger(0.25).whileTrue(a.grab());
            controller1.leftTrigger(0.25).whileTrue(a.realese());
        });

        controller2.leftTrigger(0.25).whileTrue(pickUpLeft());
        controller2.rightTrigger(0.25).whileTrue(pickUpRight());

        tusks.ifPresent(t -> {
            elevator.ifPresent(e -> {
                (controller2.leftStick().or(() -> false /* Robot is aligned at reef point */))
                        .and(() -> e.getState().isReefTargetHeight())
                        .onTrue(deployCoral());
            });
        });

        controller1.a().whileTrue(Commands.parallel(Autos.dtmToReef(), lights.dtm()));
        controller1.b().whileTrue(Commands.parallel(Autos.dtmToHumanPlayerStation(), lights.dtm()));

        controller1.povDown()
                .whileTrue(new CharacterizeDrive(swerveDrive, Volts.per(Second).ofNative(1), Seconds.of(4.0)));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(swerveDrive.runOnce(() -> swerveDrive.seedFieldCentric()));
    }

    public Command pickUpLeft() {
        if (tusks.isEmpty())
            return Commands.none();
        return Commands.parallel(tusks.get().pickup(), lights.pickUpLeft());
    }

    public Command pickUpRight() {
        if (tusks.isEmpty())
            return Commands.none();
        return Commands.parallel(tusks.get().pickup(), lights.pickUpRight());
    }

    public void updateGoalPoseField() {
        goalPoseField.setRobotPose(Autos.getDTMtoReefStartPose().orElse(new Pose2d()));
    }

    class CommandWrapper {
        public Command command = Commands.none();
    }

    public Command deployCoral() {

        CommandWrapper commander = new CommandWrapper();

        tusks.ifPresent(t -> {
            elevator.ifPresent(e -> {
                Supplier<Boolean> getTusksStartDeploying = () -> e.getState().isNearTargetHeight();
                Supplier<Boolean> getElevatorMoveToDeploy = () -> t.getAngle().lt(Degrees.of(50));

                var targetHeight = e.getState().getSelectedTargetHeight();

                commander.command = Commands.parallel(t.deploy(getTusksStartDeploying),
                        e.raiseToHeight(targetHeight, getElevatorMoveToDeploy)).until(() -> !t.getState().hasCoral());
            });
        });

        return commander.command;
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
