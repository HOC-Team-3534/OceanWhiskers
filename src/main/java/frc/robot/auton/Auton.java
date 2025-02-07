package frc.robot.auton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.reefscape.FieldAndTags2025.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
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
import lombok.Getter;
import lombok.Setter;

public class Auton {
    public static final EventTrigger autonL4 = new EventTrigger("L4");

    private static final Swerve swerve = Robot.getSwerve();

    public static class AutonConfig {
        @Getter @Setter
        private PathConstraints pathConstraints =
                new PathConstraints(
                        MetersPerSecond.of(3.0),
                        MetersPerSecondPerSecond.of(4.0),
                        RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        @Getter @Setter
        private PathConstraints endSlowPathConstraints =
                new PathConstraints(
                        MetersPerSecond.of(1.0),
                        MetersPerSecondPerSecond.of(1.0),
                        RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        @Getter @Setter private double percentSlowEndOfPath = 0.2;

        @Getter @Setter
        private Distance offsetFromWallToCenter = Inches.of(17.0); // TODO: measure and change
    }

    private Command m_autonomousCommand;

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private AutonConfig config;

    public Auton(AutonConfig config) {
        this.config = config;

        autonChooser.setDefaultOption("No Auton", Commands.none());
        autonChooser.addOption("Drive Forward", driveForward(Feet.of(2)));

        SmartDashboard.putData("Auton/Auton Chooser", autonChooser);
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
    public Command driveForward(Distance distance) {
        return AutoBuilder.followPath(createPath(getPose(), getDriveForwardGoalPose(distance)));
    }

    public static Pose2d getDriveForwardGoalPose(Distance distance) {
        return new Pose2d(
                getPose()
                        .getTranslation()
                        .plus(new Translation2d(distance.in(Meters), getPose().getRotation())),
                getPose().getRotation());
    }

    // Full Autonomous
    public Command autonPlace(
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

    public Command autonPickup(
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
    public Command dtmToHumanPlayerStation() {
        return followPathToAprilTagID(Auton::findClosestHumanPlayerStationID);
    }

    public Command dtmToReef() {
        return followPathToAprilTagID(Auton::findClosestReefID);
    }

    // Path Planning Helpers
    private Command followPathToAprilTagID(Supplier<Optional<Integer>> tagIdSupplier) {
        return Commands.deferredProxy(
                () ->
                        tagIdSupplier
                                .get()
                                .flatMap(this::findGoalPoseInFrontOfTag)
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

    private PathPlannerPath createPath(Pose2d... poses) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        return new PathPlannerPath(
                waypoints,
                Collections.emptyList(),
                Collections.emptyList(),
                List.of(
                        new ConstraintsZone(
                                ((double) (poses.length - 1)) - config.getPercentSlowEndOfPath(),
                                poses.length - 1,
                                config.getEndSlowPathConstraints())),
                Collections.emptyList(),
                config.pathConstraints,
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

    private Optional<Pose2d> findGoalPoseInFrontOfTag(int id) {
        return APRIL_TAG_FIELD_LAYOUT
                .getTagPose(id)
                .map(
                        p -> {
                            var tagPose = p.toPose2d();
                            var goalTranslation =
                                    tagPose.getTranslation()
                                            .plus(
                                                    new Translation2d(
                                                            config.getOffsetFromWallToCenter()
                                                                    .in(Meters),
                                                            tagPose.getRotation()));
                            var goalRotation =
                                    tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
                            return new Pose2d(goalTranslation, goalRotation);
                        });
    }

    private static Optional<Integer> findClosestHumanPlayerStationID() {
        if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1)) || !isRobotOnOurSide())
            return Optional.empty();

        return Optional.of(
                isRedAlliance() ? isRobotOnHighSide() ? 2 : 1 : isRobotOnHighSide() ? 13 : 12);
    }

    private static Optional<Integer> findClosestReefID() {
        if (!isRobotOnOurSide() || getAllianceReefTags().isEmpty()) return Optional.empty();

        return Optional.of(
                getAllianceReefTags().stream()
                        .min(
                                (t1, t2) -> {
                                    var dist1 = findDistanceFromRobot(t1.pose);
                                    var dist2 = findDistanceFromRobot(t2.pose);

                                    return (int) Math.round(dist1.minus(dist2).in(Meters) * 1000);
                                })
                        .get()
                        .ID);
    }
}
