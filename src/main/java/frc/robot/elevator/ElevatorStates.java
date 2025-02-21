package frc.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotStates.*;
import static frc.robot.RobotStates.ElevatorRelated.*;

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
        GoToL1Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L1Pre));
        GoToL1Coral.and(Deploy).whileTrue(goToLevel(Level.L1));

        GoToL2Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L2Pre));
        GoToL2Coral.and(Deploy).whileTrue(goToLevel(Level.L2));

        GoToL3Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L3Pre));
        GoToL3Coral.and(Deploy).whileTrue(goToLevel(Level.L3));

        GoToL4Coral.and(Deploy.not()).whileTrue(goToLevel(Level.L4Pre));
        GoToL4Coral.and(Deploy).whileTrue(goToLevel(Level.L4));

        PickupCoralLeft.or(PickupCoralRight).whileTrue(goToLevel(Level.PickUp));

        RequestJawsClosed.not()
                .and(JawsRelated.Opened.not())
                // No other command is setting the target height above 0
                .and(
                        () ->
                                elevator.getTargetHeight().isEquivalent(Inches.zero())
                                        && !elevator.getState().isClimbing())
                .onTrue(goToLevel(Level.Jaws).until(JawsRelated.Opened));

        VoltageUp.whileTrue(voltageOut(() -> Volts.of(1.5)));
        VoltageDown.whileTrue(voltageOut(() -> Volts.of(0.2)));

        QuasiasticUp.whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        QuasiasticDown.whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        DynamicUp.whileTrue(elevator.sysIdDynamic(Direction.kForward));
        DynamicDown.whileTrue(elevator.sysIdDynamic(Direction.kReverse));

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
                                            + Math.round(elevator.getVoltage().in(Volts) * 100)
                                                    / 100.0);
                    elevator.setVoltageOut(volts.get());
                });
    }
}
