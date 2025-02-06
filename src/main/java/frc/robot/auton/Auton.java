package frc.robot.auton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.Level;
import frc.robot.swerve.Swerve;
import frc.robot.tusks.Tusks.Side;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Auton {
    public static final EventTrigger autonL4 = new EventTrigger("L4");

    private static final Swerve swerve = Robot.getSwerve();

    public static class AutonConfig {}

    // Constants
    private static final AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final Distance FIELD_LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());
    private static final Distance FIELD_WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());
    private static final Distance ROBOT_BUMPERS_TO_CENTER =
            Inches.of(17.0); // TODO: measure and change

    // April Tag Management
    private static final List<AprilTag> sortedTags =
            aprilTagFieldLayout.getTags().stream().sorted((t1, t2) -> t1.ID - t2.ID).toList();
    private static final List<AprilTag> blueReefTags = sortedTags.subList(16, 22);
    private static final List<AprilTag> redReefTags = sortedTags.subList(5, 11);

    static final PathConstraints CONSTRAINTS =
            new PathConstraints(
                    MetersPerSecond.of(3.0),
                    MetersPerSecondPerSecond.of(4.0),
                    RotationsPerSecond.of(1.5),
                    RotationsPerSecondPerSecond.of(4.5));

    private Command m_autonomousCommand;

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private AutonConfig config;

    public Auton(AutonConfig config) {
        this.config = config;
        autonChooser.setDefaultOption("No Auton", Commands.none());
        autonChooser.addOption("Drive Forward", driveForward(Feet.of(2)));
        SmartDashboard.putData(autonChooser);
    }

    public void init() {
        m_autonomousCommand = getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    // Drive forward
    public static Command driveForward(Distance distance) {
        return AutoBuilder.followPath(createPath(getPose(), getDriveForwardGoalPose(distance)));
    }

    public static Pose2d getDriveForwardGoalPose(Distance distance) {
        return new Pose2d(
                getPose()
                        .getTranslation()
                        .plus(new Translation2d(distance.in(Meters), getPose().getRotation())),
                getPose().getRotation());
    }

    public enum ReefSide {
        Front(7, 18),
        FrontRight(8, 17),
        BackRight(9, 22),
        Back(10, 21),
        BackLeft(11, 20),
        FrontLeft(6, 19);
        int redID, blueID;

        ReefSide(int redID, int blueID) {
            this.redID = redID;
            this.blueID = blueID;
        }

        Optional<Integer> getID() {
            return DriverStation.getAlliance()
                    .map(alliance -> alliance.equals(Alliance.Red) ? redID : blueID);
        }
    }

    public enum PickupSide {
        Left(1, 13),
        Right(2, 12);
        int redId, blueId;

        PickupSide(int redId, int blueId) {
            this.redId = redId;
            this.blueId = blueId;
        }

        Optional<Integer> getID() {
            return DriverStation.getAlliance()
                    .map(alliance -> alliance.equals(Alliance.Red) ? redId : blueId);
        }
    }

    // Full Autonomous
    public static Command autonPlace(
            List<Pair<ReefSide, Level>> placeList, List<Pair<PickupSide, Side>> pickupList) {
        if (placeList.size() <= 0) return Commands.none();
        var options = placeList.remove(0);
        var reefSide = options.getFirst();
        var elevatorLevel = options.getSecond();

        if (reefSide.getID().isEmpty()) return Commands.none();

        return followPathToAprilTagID(reefSide::getID)
                .andThen(Commands.none())
                .andThen(autonPickup(placeList, pickupList));
    }

    public static Command autonPickup(
            List<Pair<ReefSide, Level>> placeList, List<Pair<PickupSide, Side>> pickupList) {
        if (pickupList.size() <= 0) return Commands.none();
        var options = pickupList.remove(0);
        var pickupSide = options.getFirst();
        var tusksSide = options.getSecond();

        if (pickupSide.getID().isEmpty()) return Commands.none();

        return followPathToAprilTagID(pickupSide::getID)
                .andThen(Commands.none())
                .andThen(autonPlace(placeList, pickupList));
    }

    // DTM
    public static Command dtmToHumanPlayerStation() {
        return followPathToAprilTagID(Auton::findClosestHumanPlayerStationID);
    }

    public static Command dtmToReef() {
        return followPathToAprilTagID(Auton::findClosestReefID);
    }

    // Path Planning Helpers
    private static Command followPathToAprilTagID(Supplier<Optional<Integer>> tagIdSupplier) {
        return Commands.deferredProxy(
                () ->
                        tagIdSupplier
                                .get()
                                .flatMap(Auton::findGoalPoseInFrontOfTag)
                                .map(
                                        goalPose -> {
                                            var startHeading =
                                                    getRobotDriveDirection()
                                                            .orElse(
                                                                    goalPose.getTranslation()
                                                                            .minus(
                                                                                    getPose()
                                                                                            .getTranslation())
                                                                            .getAngle());

                                            var startPose =
                                                    new Pose2d(
                                                            getPose().getTranslation(),
                                                            startHeading);
                                            var path = createPath(startPose, goalPose);

                                            return AutoBuilder.followPath(path);
                                        })
                                .orElse(Commands.none()));
    }

    public static Optional<Pose2d> getDTMtoReefStartPose() {
        return getDTMtoReefGoalPose()
                .map(
                        goalPose -> {
                            var startHeading =
                                    getRobotDriveDirection()
                                            .orElse(
                                                    goalPose.getTranslation()
                                                            .minus(getPose().getTranslation())
                                                            .getAngle());
                            return new Pose2d(getPose().getTranslation(), startHeading);
                        });
    }

    public static Optional<Pose2d> getDTMtoReefGoalPose() {
        return findClosestReefID().flatMap(Auton::findGoalPoseInFrontOfTag);
    }

    private static PathPlannerPath createPath(Pose2d... poses) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        var endSlowContraints =
                new PathConstraints(
                        MetersPerSecond.of(1.0),
                        MetersPerSecondPerSecond.of(1.0),
                        RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        return new PathPlannerPath(
                waypoints,
                Collections.emptyList(),
                Collections.emptyList(),
                List.of(
                        new ConstraintsZone(
                                ((double) (poses.length - 1)) - 0.2,
                                poses.length - 1,
                                endSlowContraints)),
                Collections.emptyList(),
                CONSTRAINTS,
                null,
                new GoalEndState(0.0, poses[poses.length - 1].getRotation()),
                false);
    }

    // Position and Alliance Helpers
    private static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance.equals(Alliance.Red))
                .orElse(false);
    }

    private static boolean isRobotOnOurSide() {
        var distanceFromAllianceWall =
                isRedAlliance()
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
        return swerve.getState().Pose;
    }

    private static Optional<Rotation2d> getRobotDriveDirection() {
        return swerve.getRobotDriveDirection();
    }

    // April Tag Utilities
    private static Distance findDistanceFromRobot(Pose3d tag) {
        return Meters.of(
                tag.getTranslation().toTranslation2d().getDistance(getPose().getTranslation()));
    }

    private static Optional<Pose2d> findGoalPoseInFrontOfTag(int id) {
        return aprilTagFieldLayout
                .getTagPose(id)
                .map(
                        p -> {
                            var tagPose = p.toPose2d();
                            var goalTranslation =
                                    tagPose.getTranslation()
                                            .plus(
                                                    new Translation2d(
                                                            ROBOT_BUMPERS_TO_CENTER.in(Meters),
                                                            tagPose.getRotation()));
                            var goalRotation =
                                    tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
                            return new Pose2d(goalTranslation, goalRotation);
                        });
    }

    private static Optional<Integer> findClosestHumanPlayerStationID() {
        if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1)) || !isRobotOnOurSide()) {
            return Optional.empty();
        }
        return Optional.of(
                isRedAlliance() ? isRobotOnHighSide() ? 2 : 1 : isRobotOnHighSide() ? 13 : 12);
    }

    private static Optional<Integer> findClosestReefID() {
        if (!isRobotOnOurSide()) {
            return Optional.empty();
        }
        if (getAllianceReefTags().isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(
                getAllianceReefTags().stream()
                        .min(
                                (t1, t2) ->
                                        (int)
                                                Math.round(
                                                        findDistanceFromRobot(t1.pose)
                                                                        .minus(
                                                                                findDistanceFromRobot(
                                                                                        t2.pose))
                                                                        .in(Meters)
                                                                * 1000))
                        .get()
                        .ID);
    }
}
