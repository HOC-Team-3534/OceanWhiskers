package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;
import frc.robot.RobotContainer;
import java.util.*;
import java.util.function.Supplier;

public final class Autos {
        // Constants
        private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.kDefaultField);
        private static final Distance FIELD_LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());
        private static final Distance FIELD_WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());
        private static final Distance ROBOT_BUMPERS_TO_CENTER = Inches.of(16.0); // TODO: measure and change

        // April Tag Management
        private static final List<AprilTag> sortedTags = aprilTagFieldLayout.getTags().stream()
                        .sorted((t1, t2) -> t1.ID - t2.ID)
                        .toList();
        private static final List<AprilTag> blueTags = sortedTags.subList(11, 22);
        private static final List<AprilTag> redTags = sortedTags.subList(0, 12);

        // Public Commands
        public static Command driveForward(Distance distance) {
                var startPose = getPose();
                var goalPose = new Pose2d(
                                startPose.getTranslation()
                                                .plus(new Translation2d(distance.in(Meters), startPose.getRotation())),
                                startPose.getRotation());

                return AutoBuilder.followPath(createPath(startPose, goalPose));
        }

        public static Command dtmToHumanPlayerStation() {
                return dtm(Autos::findClosestHumanPlayerStationID, Feet.of(2.0));
        }

        public static Command dtmToReef() {
                return dtm(Autos::findClosestReefID, Feet.of(2.0));
        }

        // Path Planning Helpers
        private static Command dtm(Supplier<Optional<Integer>> tagIdSupplier, Distance endPathDistance) {
                return tagIdSupplier.get()
                                .flatMap(Autos::findGoalPoseInFrontOfTag)
                                .map(goalPose -> {
                                        var endPath = makeStraightPathToGoal(endPathDistance, goalPose);
                                        var constraints = PathConstraints.unlimitedConstraints(12.0);
                                        return AutoBuilder.pathfindThenFollowPath(endPath, constraints);
                                })
                                .orElse(Commands.none());
        }

        private static PathPlannerPath createPath(Pose2d... poses) {
                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
                PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
                return new PathPlannerPath(
                                waypoints,
                                constraints,
                                null,
                                new GoalEndState(0.0, poses[poses.length - 1].getRotation()));
        }

        private static PathPlannerPath makeStraightPathToGoal(Distance distance, Pose2d goalPose) {
                var startPose = new Pose2d(
                                goalPose.getTranslation()
                                                .minus(new Translation2d(distance.in(Meters), goalPose.getRotation())),
                                goalPose.getRotation());
                return createPath(startPose, goalPose);
        }

        // Position and Alliance Helpers
        private static boolean isRedAlliance() {
                return DriverStation.getAlliance()
                                .map(alliance -> alliance.equals(Alliance.Red))
                                .orElse(false);
        }

        private static boolean isRobotOnOurSide() {
                var distanceFromAllianceWall = isRedAlliance()
                                ? FIELD_LENGTH.minus(getPose().getMeasureX())
                                : getPose().getMeasureX();
                return distanceFromAllianceWall.lte(FIELD_LENGTH.div(2).plus(Feet.of(1)));
        }

        private static boolean isRobotOnHighSide() {
                return getPose().getMeasureY().gt(FIELD_WIDTH.div(2));
        }

        private static List<AprilTag> getAllianceTags() {
                return DriverStation.getAlliance()
                                .map(alliance -> alliance.equals(Alliance.Red) ? redTags : blueTags)
                                .orElse(List.of());
        }

        private static Pose2d getPose() {
                return RobotContainer.getSwerveDriveSubsystem().getState().Pose;
        }

        // April Tag Utilities
        private static Distance findDistanceFromRobot(Pose3d tag) {
                return Meters.of(tag.getTranslation().toTranslation2d().getDistance(getPose().getTranslation()));
        }

        private static Optional<Pose2d> findGoalPoseInFrontOfTag(int id) {
                return aprilTagFieldLayout.getTagPose(id).map(p -> {
                        var tagPose = p.toPose2d();
                        var goalTranslation = tagPose.getTranslation().plus(
                                        new Translation2d(ROBOT_BUMPERS_TO_CENTER.in(Meters), tagPose.getRotation()));
                        var goalRotation = tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
                        return new Pose2d(goalTranslation, goalRotation);
                });
        }

        private static Optional<Integer> findClosestHumanPlayerStationID() {
                if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1)) || !isRobotOnOurSide()) {
                        return Optional.empty();
                }
                return Optional.of(isRedAlliance()
                                ? isRobotOnHighSide() ? 2 : 1
                                : isRobotOnHighSide() ? 13 : 12);
        }

        private static Optional<Integer> findClosestReefID() {
                if (!isRobotOnOurSide()) {
                        return Optional.empty();
                }
                return Optional.of(getAllianceTags().stream()
                                .min((t1, t2) -> (int) Math.round(
                                                findDistanceFromRobot(t1.pose)
                                                                .minus(findDistanceFromRobot(t2.pose))
                                                                .in(Meters) * 1000))
                                .get().ID);
        }
}