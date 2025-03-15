package frc.robot.subsystems.door;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class DoorStates {
    private static Door door = Robot.getDoor();

    public static void setupDefaultCommand() {}

    public static void setupBindings() {
        var ForbarIn = new Trigger(() -> Robot.getForbar().getState().isIn());
        var ForbarOut = new Trigger(() -> Robot.getForbar().getState().isOut());

        var DoorIn = new Trigger(() -> door.getState().isIn());

        ForbarOut.and(GoToL1Coral.not()).onTrue(door.zero());
        ForbarOut.and(GoToL1Coral).onTrue(door.out());
        DoorIn.onTrue(door.holdIn());
        DoorIn.not().and(ForbarIn).onTrue(door.in());
    }
}
