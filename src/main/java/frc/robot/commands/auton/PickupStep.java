package frc.robot.commands.auton;

import static frc.robot.commands.auton.AutonChoosers.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.robot.Robot;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FollowPathThenDriveToPose;
import frc.robot.subsystems.swerve.Swerve;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PickupStep extends AutonStep {
    final ReefBranch prevBranch, branch;

    private Timer leavePickupTimeout = new Timer();

    @Override
    public PathPlannerPath getPath() {
        var paths = Paths.getReefSidePaths(prevBranch.getReefSide());
        if (isLeftSideOfFieldSelected()) return paths.getToLeft();
        else return paths.getToRight();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Command followPath() {
        return new FollowPathThenDriveToPose<Swerve>(
                (FollowPathCommand) super.followPath(),
                (DriveToPose<Swerve>) super.alignWithGoalPose());
    }

    @Override
    public boolean isStepComplete() {
        return Robot.getTusks().getState().isHoldingCoral() || leavePickupTimeout.hasElapsed(2.0);
    }

    public ReefBranch.Side getSide() {
        return branch.getSide();
    }

    @Override
    public Command alignWithGoalPose() {
        return Commands.none();
    }
}
