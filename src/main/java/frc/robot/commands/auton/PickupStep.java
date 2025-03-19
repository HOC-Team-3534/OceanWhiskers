package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Feet;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.Robot;
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

    @Override
    public Command followPath() {
        return new FollowPathThenDriveToPose<Swerve>(
                        (FollowPathCommand) super.followPath(),
                        (DriveToPose<Swerve>)
                                new DriveToPose<Swerve>(
                                        Robot.getSwerve(),
                                        () -> {
                                            var robot =
                                                    Robot.getSwerve().getPose().getTranslation();
                                            var goal = getGoalPose();

                                            var closeToWall =
                                                    goal.relativeTo(
                                                                    new Pose2d(
                                                                            robot,
                                                                            goal.getRotation()))
                                                            .getMeasureX()
                                                            .isNear(Feet.zero(), Feet.of(1));

                                            var baseXY =
                                                    closeToWall ? robot : goal.getTranslation();

                                            return new Pose2d(baseXY, goal.getRotation())
                                                    .transformBy(
                                                            new Transform2d(
                                                                    Units.feetToMeters(1.0),
                                                                    0.0,
                                                                    Rotation2d.kZero));
                                        }))
                .until(RobotStates.CanRangeCloseToWall);
    }

    @Override
    public boolean isStepComplete() {
        return RobotStates.ForbarHoldingCoralDebounce.getAsBoolean()
                || leavePickupTimeout.hasElapsed(2.0);
    }

    @Override
    public Command alignWithGoalPose() {
        return Commands.none();
    }
}
