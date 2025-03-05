package frc.robot.commands.auton;

import static frc.robot.commands.auton.AutonChoosers.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.tusks.Tusks;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PickupStep extends AutonStep {
    final ReefBranch prevBranch, branch;

    private Timer leavePickupTimeout = new Timer();

    @Override
    public PathPlannerPath getPath() {
        var reefSide = prevBranch.getReefSide();
        if (isLeftSideOfFieldSelected()) return reefSide.getToLeft();
        else return reefSide.getToRight();
    }

    @Override
    public boolean isStepComplete() {
        return Robot.getTusks().getState().isHoldingCoral() || leavePickupTimeout.hasElapsed(2.0);
    }

    public Tusks.Side getTusksSide() {
        return branch.getTusksSide();
    }

    @Override
    public Command alignWithGoalPose() {
        return super.alignWithGoalPose()
                .andThen(
                        Commands.runOnce(() -> leavePickupTimeout.restart()),
                        Robot.getDtm().pushForwardAgainstWallPickup().asProxy().withTimeout(0.5));
    }
}
