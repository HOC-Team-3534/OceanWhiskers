package frc.robot.auton;

import static frc.robot.auton.AutonChoosers.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.tusks.Tusks;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PickupStep extends AutonStep {
    final ReefBranch prevBranch, branch;

    @Override
    public PathPlannerPath getPath() {
        var reefSide = prevBranch.getReefSide();
        if (isLeftSideOfFieldSelected()) return reefSide.getToLeft();
        else return reefSide.getToRight();
    }

    @Override
    public boolean isStepComplete() {
        return Robot.getTusks().getState().isHoldingCoral();
    }

    public Tusks.Side getTusksSide() {
        return branch.getTusksSide();
    }

    @Override
    public Command alignWithGoalPose() {
        return Robot.getDtm().pushForwardAgainstWallPickup().asProxy().withTimeout(0.5);
    }
}
