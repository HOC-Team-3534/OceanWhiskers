package frc.robot.subsystems.door;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class DoorStates {
    private static Door door = Robot.getDoor();

    static Trigger ForbarIn = new Trigger(() -> Robot.getForbar().getState().isIn());
    static Trigger ForbarOut = new Trigger(() -> Robot.getForbar().getState().isOut());
    static Trigger DoorIn = new Trigger(() -> door.getState().isIn());

    static Trigger AttemptingToPickup = AlignedForPickup.and(ForbarHoldingCoral.not());
    static Trigger ScoringCoral = ForbarOut.debounce(0.15);

    public static void setupDefaultCommand() {
        door.setDefaultCommand(
                Commands.either(
                        door.out(),
                        Commands.either(door.holdIn(), door.zero(), () -> DoorIn.getAsBoolean()),
                        () ->
                                AlignedForPickup.getAsBoolean()
                                        && !ForbarHoldingCoral.getAsBoolean()));
    }

    public static void setupBindings() {
        ScoringCoral.onTrue(door.out());
        DoorIn.not().and(ForbarIn, AttemptingToPickup.not()).onTrue(door.in());
    }
}
