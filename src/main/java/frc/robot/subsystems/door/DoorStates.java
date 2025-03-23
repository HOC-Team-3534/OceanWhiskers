package frc.robot.subsystems.door;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.util.Util;
import frc.robot.Robot;
import frc.robot.RobotStates;

public class DoorStates {
    private static Door door = Robot.getDoor();

    static Trigger ForbarIn = new Trigger(() -> Robot.getForbar().getState().isIn());
    static Trigger ForbarOut = new Trigger(() -> Robot.getForbar().getState().isOut());
    static Trigger DoorIn = new Trigger(() -> door.getState().isIn());
    static Trigger DoorOut = new Trigger(() -> door.getState().isOut());

    static Trigger AttemptingToPickup =
            CanRangeCloseToWall.and(Util.autoMode)
                    .debounce(0.35)
                    .latchWithReset(
                            CanRangeAwayFromWall.or(RobotStates.ForbarHoldingCoral, Util.teleop));
    static Trigger ScoringCoral =
            ForbarOut.debounce(0.15)
                    .and(
                            Util.teleop.or(
                                    Util.autoMode.and(RobotStates::isAlignedWithReefForDeployment)))
                    .or(GoToL1Coral);

    public static void setupDefaultCommand() {
        door.setDefaultCommand(
                Commands.either(
                        Commands.either(door.zero(), door.out(), DoorOut),
                        Commands.either(door.holdIn(), door.zero(), () -> DoorIn.getAsBoolean()),
                        AttemptingToPickup));
    }

    public static void setupBindings() {
        ScoringCoral.onTrue(door.out());
        DoorIn.not().and(AttemptingToPickup.not(), ScoringCoral.not()).onTrue(door.in());
    }
}
