package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.reefscape.FieldAndTags2025.APRIL_TAG_FIELD_LAYOUT;
import static frc.reefscape.FieldAndTags2025.FIELD_WIDTH;
import static frc.reefscape.FieldAndTags2025.getAllianceReefTags;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.GeomUtil;
import frc.hocLib.util.Util;
import frc.reefscape.FieldAndTags2025.LoadingStation;
import frc.reefscape.FieldAndTags2025.ReefSide;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.commands.DriveToPose;
import frc.robot.controllers.Driver;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FieldUtil;
import java.util.Comparator;
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

    private final Driver driver = Robot.getDriver();

    public Command driveToPose(Supplier<Pose2d> target) {
        return driveToPose(target, () -> Robot.getSwerve().getPose());
    }

    public Command driveToPose(Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        return Commands.deadline(
                new DriveToPose<Swerve>(
                                Robot.getSwerve(),
                                target,
                                robot,
                                () -> {
                                    var input =
                                            new Translation2d(
                                                    driver.getDriveFwdPositive(),
                                                    driver.getDriveLeftPositive());

                                    return Util.isRedAlliance() ? input.unaryMinus() : input;
                                },
                                () -> {
                                    var input = driver.getDriveCCWPositive();

                                    return Util.isRedAlliance() ? -input : input;
                                })
                        .asProxy(),
                Commands.startEnd(
                        () -> RobotStates.setDrivingAutonomously(true),
                        () -> RobotStates.setDrivingAutonomously(false)));
    }

    public Command dtmToHumanPlayerStation() {
        var command =
                driveToPose(
                        () ->
                                getClosestHumanPlayerStationID()
                                        .flatMap(this::findGoalPoseInFrontOfTag)
                                        .orElseGet(() -> Robot.getSwerve().getPose()));

        command.setName("DTM TO PICKUP STATION");

        return command;
    }

    public Command dtmToReef() {
        return Commands.deferredProxy(
                () -> getClosestReefID().map(this::dtmToReef).orElse(Commands.none()));
    }

    public Command dtmToReef(int tagId) {
        var goalPose = findGoalPoseInFrontOfTag(tagId);

        if (goalPose.isEmpty()) return Commands.none();

        var command =
                driveToPose(
                                goalPose::get,
                                () ->
                                        Robot.getVisionSystem()
                                                .getPoseEstimateByTag(tagId)
                                                .orElseGet(() -> Robot.getSwerve().getPose()))
                        .andThen(
                                Commands.runOnce(
                                        () -> RobotStates.setAlignedWithReefForDeployment(true)));

        command.setName("DTM TO REEF");

        return command;
    }

    public DriveToPose<Swerve> driveToReefSide(ReefSide reefSide) {
        return new DriveToPose<Swerve>(
                Robot.getSwerve(),
                () -> findGoalPoseInFrontOfTag(reefSide.getTagId()).get(),
                () ->
                        Robot.getVisionSystem()
                                .getPoseEstimateByTag(reefSide.getTagId())
                                .orElseGet(() -> Robot.getSwerve().getPose()));
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

    private static CachedValue<Optional<Integer>> cachedClosestHumanPlayerStation =
            new CachedValue<>(DTM::updateClosestHumanPlayerStationID);
    private static CachedValue<Optional<Integer>> cachedClosestReef =
            new CachedValue<>(DTM::updateClosestReefID);

    private static Optional<Integer> updateClosestHumanPlayerStationID() {
        var currentPose = Robot.getSwerve().getPose();

        if (currentPose.getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1))
                || !FieldUtil.isRobotOnOurSide(currentPose)) return Optional.empty();

        return Optional.of(
                LoadingStation.fromSide(FieldUtil.getCurrentSide(currentPose)).getTagId());
    }

    public static Optional<Integer> getClosestHumanPlayerStationID() {
        return cachedClosestHumanPlayerStation.get();
    }

    private static Optional<Integer> updateClosestReefID() {
        var currentPose = Robot.getSwerve().getPose();

        if (!FieldUtil.isRobotOnOurSide(currentPose) || getAllianceReefTags().isEmpty())
            return Optional.empty();

        return getAllianceReefTags().stream()
                .min(
                        Comparator.comparingDouble(
                                tag -> GeomUtil.calc2dDistance(currentPose, tag.pose).in(Meters)))
                .map(tag -> tag.ID);
    }

    public static Optional<Integer> getClosestReefID() {
        return cachedClosestReef.get();
    }
}
