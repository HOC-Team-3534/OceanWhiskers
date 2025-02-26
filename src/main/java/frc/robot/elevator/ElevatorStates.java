package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotStates.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.Level;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(goToLevel(Level.Bottom));
    }

    public static void setupBindings() {
        GoToL1Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L1Pre));
        GoToL1Coral.and(Deploy).whileTrue(goToLevel(Level.L1));

        GoToL2Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L2Pre));
        GoToL2Coral.and(Deploy).whileTrue(goToLevel(Level.L2));

        GoToL3Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L3Pre));
        GoToL3Coral.and(Deploy).whileTrue(goToLevel(Level.L3));

        GoToL4Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L4Pre));
        GoToL4Coral.and(Deploy).whileTrue(goToLevel(Level.L4));

        GoToL2Algae.whileTrue(goToLevel(Level.L2Algae));
        GoToL3Algae.whileTrue(goToLevel(Level.L3Algae));

        PreClimb.whileTrue(goToLevel(Level.PreClimb));

        Climb.and(
                        () ->
                                (elevator.getState().isNearTargetHeight()
                                                && elevator.getState()
                                                        .getTargetLevel()
                                                        .equals(Level.PreClimb))
                                        || elevator.getState().isClimbing())
                .whileTrue(elevator.climb());

        PickupCoralLeft.or(PickupCoralRight).whileTrue(goToLevel(Level.PickUp));

        RequestJawsClosed.not()
                .and(JawsOpened.not())
                // No other command is setting the target height above 0
                .and(
                        () ->
                                elevator.getState().getTargetLevel().equals(Level.Bottom)
                                        && !elevator.getState().isClimbing())
                .onTrue(goToLevel(Level.Jaws).until(JawsOpened));

        ElevatorVoltageUp.whileTrue(
                voltageOut(
                        () ->
                                Volts.of(
                                        SmartDashboard.getNumber(
                                                "Elevator/Voltage Up Command", 1.1))));
        ElevatorVoltageDown.whileTrue(
                voltageOut(
                        () ->
                                Volts.of(
                                        SmartDashboard.getNumber(
                                                "Elevator/Voltage Down Command", 0.6))));

        ElevatorQuasiasticUp.whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        ElevatorQuasiasticDown.whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        ElevatorDynamicUp.whileTrue(elevator.sysIdDynamic(Direction.kForward));
        ElevatorDynamicDown.whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    }

    static Command goToLevel(Level level) {
        return elevator.goToLevel(level);
    }

    static Command voltageOut(Supplier<Voltage> volts) {
        return elevator.run(
                () -> {
                    elevator.getCurrentCommand()
                            .setName(
                                    "Elevator.Apply Voltage Out - "
                                            + Math.round(elevator.getVoltage().in(Volts) * 100)
                                                    / 100.0);
                    elevator.setVoltageOut(volts.get());
                });
    }
}
