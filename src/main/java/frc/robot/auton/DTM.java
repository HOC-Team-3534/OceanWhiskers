package frc.robot.auton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.reefscape.FieldAndTags2025.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.util.CachedValue;
import frc.reefscape.FieldAndTags2025;
import frc.robot.Robot;
import frc.robot.RobotStates;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class DTM {
    @Getter
    @Setter
    @Accessors(chain = true)
    public static class DTMConfig {
        PathConstraints pathConstraints =
                new PathConstraints(
                        MetersPerSecond.of(3.0),
                        MetersPerSecondPerSecond.of(4.0),
                        RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        Distance offsetFromWallToCenter = Inches.of(16.5);

        // TODO: tune skip and alignment values
        Distance skipPathForDTMTolerance = Inches.of(0.0); // taken out so never skip
        Angle skipPathFromDTMAngleTolerance = Degrees.of(10.0);

        Distance offsetFromWallToCenterDTM = Inches.of(-16.5);

        Pose2d dtmAlignTolerance =
                new Pose2d(Inches.of(3.25), Inches.of(1.0), Rotation2d.fromDegrees(4.0));

        Time pushAgainstWallTimeReef = Seconds.of(0.40);
        Time pushAgainstWallTimePickup = Seconds.of(0.3);
    }

    private final DTMConfig config;

    public DTM(DTMConfig config) {
        this.config = config;
    }

    public Command dtmToHumanPlayerStation() {
        var command =
                Commands.deadline(
                        followPathToAprilTagID(FieldAndTags2025::getClosestHumanPlayerStationID)
                                .andThen(pushForwardAgainstWallPickup().asProxy().withTimeout(0.5)),
                        Commands.startEnd(
                                () -> RobotStates.setDrivingAutonomously(true),
                                () -> RobotStates.setDrivingAutonomously(false)));

        command.setName("DTM TO PICKUP STATION");

        return command;
    }

    public Command dtmToReef() {
        var command =
                Commands.deadline(
                        followPathToAprilTagID(FieldAndTags2025::getClosestReefID)
                                .andThen(alignLeftRightOnReefWall().asProxy()),
                        Commands.startEnd(
                                () -> RobotStates.setDrivingAutonomously(true),
                                () -> RobotStates.setDrivingAutonomously(false)));

        command.setName("DTM TO REEF");

        return command;
    }

    private Command followPathToAprilTagID(Supplier<Optional<Integer>> tagIdSupplier) {
        // TODO: test dtm, making sure precise alignment works
        return Commands.deferredProxy(
                () -> {
                    return tagIdSupplier
                            .get()
                            .flatMap(this::findGoalPoseInFrontOfTag)
                            .map(
                                    goalPose -> {
                                        var startPose =
                                                new Pose2d(
                                                        getPose().getTranslation(),
                                                        calculateDirectionToStartDrivingIn(
                                                                goalPose));

                                        if (startPose
                                                                .getTranslation()
                                                                .minus(goalPose.getTranslation())
                                                                .getNorm()
                                                        < config.getSkipPathForDTMTolerance()
                                                                .in(Meters)
                                                && startPose
                                                        .getRotation()
                                                        .minus(goalPose.getRotation())
                                                        .getMeasure()
                                                        .isNear(
                                                                Degrees.zero(),
                                                                config
                                                                        .getSkipPathFromDTMAngleTolerance())) {
                                            return Commands.none();
                                        }

                                        var path =
                                                new PathPlannerPath(
                                                        PathPlannerPath.waypointsFromPoses(
                                                                startPose, goalPose),
                                                        config.pathConstraints,
                                                        null,
                                                        new GoalEndState(
                                                                0.0, goalPose.getRotation()));

                                        if (path.getAllPathPoints().size() < 5)
                                            return Commands.none();

                                        var wpilibTrajectory =
                                                TrajectoryGenerator.generateTrajectory(
                                                        List.of(startPose, goalPose),
                                                        new TrajectoryConfig(
                                                                config.pathConstraints
                                                                        .maxVelocity(),
                                                                config.pathConstraints
                                                                        .maxAcceleration()));

                                        Robot.getSwerve()
                                                .getField()
                                                .getObject("DTM Path")
                                                .setTrajectory(wpilibTrajectory);

                                        path.preventFlipping = true;

                                        return AutoBuilder.followPath(path);
                                    })
                            .orElse(Commands.none());
                });
    }

    protected Command alignLeftRightOnReefWall() {
        return Robot.getSwerve()
                .driveAgainstWallAlign(
                        this::getAlignReefFinalTransform,
                        config.getDtmAlignTolerance(),
                        config.getPushAgainstWallTimeReef())
                .asProxy()
                .until(() -> RobotStates.AlignedWithReef.getAsBoolean())
                .withTimeout(0.5)
                .andThen(
                        Commands.deferredProxy(
                                () -> {
                                    if (!RobotStates.AlignedWithReef.getAsBoolean())
                                        return alignLeftRightOnReefWall();
                                    return Commands.none();
                                }));
    }

    protected Command pushForwardAgainstWallPickup() {
        return Robot.getSwerve()
                .driveAgainstWallAlign(
                        () -> new Transform2d(Inches.of(2.0), Inches.zero(), new Rotation2d()),
                        new Pose2d(),
                        config.getPushAgainstWallTimePickup());
    }

    private CachedValue<Optional<Transform2d>> cachedBumperToReefAlignment =
            new CachedValue<>(this::updateBumperToReefAlignment);

    public Optional<Transform2d> updateBumperToReefAlignment() {
        return Robot.getVisionSystem()
                .getRobotToReefAlignment()
                .flatMap(
                        (robotToReef) -> {
                            if (getClosestReefID().isEmpty()) return Optional.empty();

                            var reefId = getClosestReefID().get();
                            var reefTargetPose =
                                    FieldAndTags2025.APRIL_TAG_FIELD_LAYOUT
                                            .getTagPose(reefId)
                                            .map(Pose3d::toPose2d);

                            if (reefTargetPose.isEmpty()) return Optional.empty();

                            var backedUpFromReef =
                                    new Transform2d(
                                            new Translation2d(
                                                    config.getOffsetFromWallToCenterDTM(),
                                                    Inches.of(0)),
                                            new Rotation2d());

                            var bumperToCenterOfRobotAwayFromReef =
                                    robotToReef.plus(backedUpFromReef);

                            return Optional.of(bumperToCenterOfRobotAwayFromReef);
                        });
    }

    public Optional<Transform2d> getBumperToReefAlignment() {
        return cachedBumperToReefAlignment.get();
    }

    public boolean isBumperToReefAligned() {
        var bumperToReef = getAlignReefFinalTransform();

        var tolerance = config.getDtmAlignTolerance();

        return getBumperToReefAlignment().isPresent()
                && bumperToReef.getMeasureY().abs(Meters)
                        < tolerance.getY() + Inches.of(0.25).in(Meters)
                && bumperToReef.getRotation().getMeasure().abs(Degrees)
                        <= tolerance.getRotation().getMeasure().in(Degrees) * 1.1;
    }

    public Optional<Pose2d> getDTMToReefGoal() {
        return getClosestReefID().flatMap(this::findGoalPoseInFrontOfTag);
    }

    private CachedValue<Transform2d> cachedAlignReefFinalTransform =
            new CachedValue<>(this::updateAlignReefFinalTransform);

    private Transform2d updateAlignReefFinalTransform() {
        return getBumperToReefAlignment()
                .orElseGet(
                        () ->
                                getDTMToReefGoal()
                                        .map(
                                                (reefGoalPose) ->
                                                        new Transform2d(getPose(), reefGoalPose))
                                        .orElse(new Transform2d()));
    }

    public Transform2d getAlignReefFinalTransform() {
        return cachedAlignReefFinalTransform.get();
    }

    private Optional<Pose2d> findGoalPoseInFrontOfTag(int id) {
        return APRIL_TAG_FIELD_LAYOUT
                .getTagPose(id)
                .map(
                        p ->
                                p.toPose2d()
                                        .plus(
                                                new Transform2d(
                                                        config.getOffsetFromWallToCenter(),
                                                        Meters.zero(),
                                                        Rotation2d.fromDegrees(180))));
    }

    private static Rotation2d calculateDirectionToStartDrivingIn(Pose2d goalPose) {
        return Robot.getSwerve()
                .getRobotDriveDirection()
                .orElse(calculateDirectionFromCurrentPose(goalPose));
    }

    private static Rotation2d calculateDirectionFromCurrentPose(Pose2d goalPose) {
        return goalPose.getTranslation().minus(getPose().getTranslation()).getAngle();
    }

    private static Pose2d getPose() {
        return Robot.getSwerve().getState().Pose;
    }
}
