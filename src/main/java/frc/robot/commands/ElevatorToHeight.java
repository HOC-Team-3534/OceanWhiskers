package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToHeight extends Command {

    private final ElevatorSubsystem elevator;
    private final Distance targetHeight;

    public ElevatorToHeight(Distance targetHeight) {
        this.targetHeight = targetHeight;
        elevator = RobotContainer.getElevatorSubsystem();
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setHeight(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltageOutToZero();
    }
}
