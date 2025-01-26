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
        private static final Distance ROBOT_BUMPERS_TO_CENTER = Inches.of(17.0); // TODO: measure and change

        // April Tag Management
        private static final List<AprilTag> sortedTags = aprilTagFieldLayout.getTags().stream()
                        .sorted((t1, t2) -> t1.ID - t2.ID)
                        .toList();
        private static final List<AprilTag> blueReefTags = sortedTags.subList(16, 22);
        private static final List<AprilTag> redReefTags = sortedTags.subList(5, 11);

        static final PathConstraints CONSTRAINTS = new PathConstraints(MetersPerSecond.of(3.0),
                        MetersPerSecondPerSecond.of(4.0), RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        // Public Commands
        public static Command driveForward(Distance distance) {
                return AutoBuilder.followPath(createPath(getPose(), getDriveForwardGoalPose(distance)));
        }

        public static Pose2d getDriveForwardGoalPose(Distance distance) {
                return new Pose2d(
                                getPose().getTranslation()
                                                .plus(new Translation2d(distance.in(Meters), getPose().getRotation())),
                                getPose().getRotation());
        }

        public static Command dtmToHumanPlayerStation() {
                return dtm(Autos::findClosestHumanPlayerStationID);
        }

        public static Command dtmToReef() {
                return dtm(Autos::findClosestReefID);
        }

        private static Optional<Rotation2d> getRobotDriveDirection() {
                var speeds = RobotContainer.getSwerveDriveSubsystem().getState().Speeds;
                var vector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                .rotateBy(getPose().getRotation());

                if (vector.getNorm() < 0.05)
                        return Optional.empty();

                return Optional.of(vector.getAngle());
        }

        // Path Planning Helpers
        private static Command dtm(Supplier<Optional<Integer>> tagIdSupplier) {
                return tagIdSupplier.get()
                                .flatMap(Autos::findGoalPoseInFrontOfTag)
                                .map(goalPose -> {
                                        var startHeading = getRobotDriveDirection()
                                                        .orElse(goalPose.getTranslation()
                                                                        .minus(getPose().getTranslation()).getAngle());

                                        var startPose = new Pose2d(getPose().getTranslation(), startHeading);
                                        var path = createPath(startPose, goalPose);
                                        return AutoBuilder.followPath(path);
                                })
                                .orElse(Commands.none());
        }

        public static Optional<Pose2d> getDTMtoReefStartPose() {
                return getDTMtoReefGoalPose().map(goalPose -> {
                        var startHeading = getRobotDriveDirection()
                                        .orElse(goalPose.getTranslation().minus(getPose().getTranslation()).getAngle());
                        return new Pose2d(getPose().getTranslation(), startHeading);
                });
        }

        public static Optional<Pose2d> getDTMtoReefGoalPose() {
                return findClosestReefID().flatMap(Autos::findGoalPoseInFrontOfTag);
        }

        private static PathPlannerPath createPath(Pose2d... poses) {
                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

                var endSlowContraints = new PathConstraints(MetersPerSecond.of(1.0),
                                MetersPerSecondPerSecond.of(1.0), RotationsPerSecond.of(1.5),
                                RotationsPerSecondPerSecond.of(4.5));

                return new PathPlannerPath(
                                waypoints,
                                Collections.emptyList(),
                                Collections.emptyList(),
                                List.of(new ConstraintsZone(((double) (poses.length - 1)) - 0.2, poses.length - 1,
                                                endSlowContraints)),
                                Collections.emptyList(),
                                CONSTRAINTS,
                                null,
                                new GoalEndState(0.0, poses[poses.length - 1].getRotation()), false);
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

        private static List<AprilTag> getAllianceReefTags() {
                return DriverStation.getAlliance()
                                .map(alliance -> alliance.equals(Alliance.Red) ? redReefTags : blueReefTags)
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
                return Optional.of(getAllianceReefTags().stream()
                                .min((t1, t2) -> (int) Math.round(
                                                findDistanceFromRobot(t1.pose)
                                                                .minus(findDistanceFromRobot(t2.pose))
                                                                .in(Meters) * 1000))
                                .get().ID);
        }
}