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
        GoToL1.and(Deploy.not()).whileTrue(goToLevel(Level.L1Pre));
        GoToL1.and(Deploy).whileTrue(goToLevel(Level.L1));

        GoToL2.and(Deploy.not()).whileTrue(goToLevel(Level.L2Pre));
        GoToL2.and(Deploy).whileTrue(goToLevel(Level.L2));

        GoToL3.and(Deploy.not()).whileTrue(goToLevel(Level.L3Pre));
        GoToL3.and(Deploy).whileTrue(goToLevel(Level.L3));

        GoToL4.and(Deploy.not()).whileTrue(goToLevel(Level.L4Pre));
        GoToL4.and(Deploy).whileTrue(goToLevel(Level.L4));

        PickupCoralLeft.or(PickupCoralRight).whileTrue(goToLevel(Level.PickUp));

        ElevatorVoltageUp.whileTrue(voltageOut(() -> Volts.of(1.5)));
        ElevatorVoltageDown.whileTrue(voltageOut(() -> Volts.of(0.2)));

        ElevatorQuasiasticUp.whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        ElevatorQuasiasticDown.whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        ElevatorDynamicUp.whileTrue(elevator.sysIdDynamic(Direction.kForward));
        ElevatorDynamicDown.whileTrue(elevator.sysIdDynamic(Direction.kReverse));

        // TODO: add elevator climb and climbing logic to keep the elevator from breaking
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
                                            + elevator.getVoltage().in(Volts));
                    elevator.setVoltageOut(volts.get());
                });
    }
}
