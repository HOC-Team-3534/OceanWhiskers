package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public final class Autos {
    private static final DistanceUnit Meters = null;

    public static Command driveForward(Distance distance) {
        var startPose = RobotContainer.getSwerveDriveSubsystem().getState().Pose;
        var goalPose = new Pose2d(
                startPose.getTranslation()
                        .plus(new Translation2d(distance.in(Meters), startPose.getRotation())),
                startPose.getRotation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, goalPose);

        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
                new GoalEndState(0.0, startPose.getRotation()));

        return AutoBuilder.followPath(path);
    }
}
