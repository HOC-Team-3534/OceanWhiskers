package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefault extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorDefault() {
        elevator = RobotContainer.getElevatorSubsystem();
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtBottomOfTravel() || elevator.getState().isClimbing())
            elevator.setVoltageOutToZero();
        else
            elevator.setHeight(Inches.of(0));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltageOutToZero();
    }
}
