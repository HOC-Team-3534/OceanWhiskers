package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotStates.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.Level;
import java.util.function.Supplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(goToLevel(Level.Bottom));
    }

    public static void setupBindings() {
        GoToL1.and(ReadyToDeploy.not()).whileTrue(goToLevel(Level.L1, () -> false));
        GoToL1.and(ReadyToDeploy).whileTrue(goToLevel(Level.L1, () -> true));

        GoToL2.and(ReadyToDeploy.not()).whileTrue(goToLevel(Level.L2, () -> false));
        GoToL2.and(ReadyToDeploy).whileTrue(goToLevel(Level.L2, () -> true));

        GoToL3.and(ReadyToDeploy.not()).whileTrue(goToLevel(Level.L3, () -> false));
        GoToL3.and(ReadyToDeploy).whileTrue(goToLevel(Level.L3, () -> true));

        GoToL4.and(ReadyToDeploy.not()).whileTrue(goToLevel(Level.L4, () -> false));
        GoToL4.and(ReadyToDeploy).whileTrue(goToLevel(Level.L4, () -> true));

        PickupCoralLeft.or(PickupCoralRight).whileTrue(goToLevel(Level.PickUp));

        ElevatorVoltageUp.whileTrue(voltageOut(() -> Volts.of(1.5)));
        ElevatorVoltageDown.whileTrue(voltageOut(() -> Volts.of(0.2)));

        ElevatorQuasiasticUp.whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        ElevatorQuasiasticDown.whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        ElevatorDynamicUp.whileTrue(elevator.sysIdDynamic(Direction.kForward));
        ElevatorDynamicDown.whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    }

    static Command goToLevel(Level level) {
        return goToLevel(level, () -> false);
    }

    static Command goToLevel(Level level, Supplier<Boolean> deploy) {
        return elevator.runEnd(
                        () -> {
                            elevator.getState().setTargetLevel(level);
                            switch (level) {
                                case Bottom:
                                    elevator.getState().setDeploying(false);
                                    break;
                                default:
                                    if (deploy.get()) {
                                        elevator.getState().setDeploying(true);
                                    }
                                    break;
                            }
                            elevator.updateHeight();
                        },
                        elevator::slowlyLower)
                .withName("Elevator.Go To " + level.name());
    }

    static Command voltageOut(Supplier<Voltage> volts) {
        return elevator.run(
                () -> {
                    elevator.getCurrentCommand()
                            .setName(
                                    "Elevator.Apply Voltage Out - "
                                            + elevator.getVoltage().in(Volts));
                    elevator.setVoltageOut(volts.get());
                });
    }
}
