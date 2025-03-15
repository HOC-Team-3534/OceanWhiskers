package frc.robot.commands.auton;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.RobotStates;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FollowPathThenDriveToPose;
import frc.robot.subsystems.swerve.Swerve;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PickupStep extends AutonStep {
    final ReefBranch prevBranch;
    final SideOfField sideOfField;

    private Timer leavePickupTimeout = new Timer();

    @Override
    public PathPlannerPath getPath() {
        var paths = Paths.getReefSidePaths(prevBranch.getReefSide());
        if (sideOfField.equals(SideOfField.Left)) return paths.getToLeft();
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
        return RobotStates.ForbarHoldingCoral.getAsBoolean() || leavePickupTimeout.hasElapsed(2.0);
    }

    @Override
    public Command alignWithGoalPose() {
        return Commands.none();
    }
}
