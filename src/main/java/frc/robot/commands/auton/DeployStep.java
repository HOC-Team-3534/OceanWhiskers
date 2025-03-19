package frc.robot.commands.auton;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.commands.FollowPathThenDriveToPose;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
@Getter
public class DeployStep extends AutonStep {
    final boolean start;
    final int level;
    final ReefBranch branch;

    final SideOfField sideOfField;

    @Getter private Timer deployTimeout = new Timer();

    @Override
    public PathPlannerPath getPath() {
        var paths = Paths.getReefSidePaths(branch.getReefSide());
        if (sideOfField.equals(SideOfField.Left)) {
            if (start) return paths.getFromStartLeft();
            else return paths.getFromLeft();
        } else {
            if (start) return paths.getFromStartRight();
            else return paths.getFromRight();
        }
    }

    @Override
    public Command followPath() {
        return new FollowPathThenDriveToPose<Swerve>(
                        (FollowPathCommand) super.followPath(),
                        Robot.getDtm().driveToReefBranch(branch))
                .asProxy()
                .andThen(Commands.runOnce(() -> RobotStates.setAlignedWithReefForDeployment(true)));
    }

    @Override
    public boolean isStepComplete() {
        return !RobotStates.ForbarHoldingCoral.getAsBoolean();
    }

    @Override
    public Command alignWithGoalPose() {
        return Commands.none();
    }
}
