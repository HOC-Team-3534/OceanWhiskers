package frc.robot.auton;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.reefscape.FieldAndTags2025.*;
import static frc.robot.auton.AutonChoosers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.util.CachedValue;
import frc.reefscape.FieldAndTags2025;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.swerve.Swerve;
import frc.robot.tusks.Tusks;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Auton {
    private static final Swerve swerve = Robot.getSwerve();
    private static final Tusks tusks = Robot.getTusks();

    static final EventTrigger autonDeploy = new EventTrigger("Deploy");
    static final EventTrigger autonPickup = new EventTrigger("Pickup");

    @Getter @Setter static Optional<AutonStep> currentStep = Optional.empty();

    public static Trigger isLevel(int level) {
        return new Trigger(() -> getCurrentStep().map(step -> step.isLevel(level)).orElse(false))
                .and(autonDeploy);
    }

    public static Trigger isTusksSide(Tusks.Side side) {
        return new Trigger(() -> getCurrentStep().map(step -> step.isTusksSide(side)).orElse(false))
                .and(autonPickup);
    }

    static {
        AutonChoosers.init();
    }

    @Getter
    @Setter
    @Accessors(chain = true)
    public static class AutonConfig {
        PathConstraints pathConstraints =
                new PathConstraints(
                        MetersPerSecond.of(3.0),
                        MetersPerSecondPerSecond.of(4.0),
                        RotationsPerSecond.of(1.5),
                        RotationsPerSecondPerSecond.of(4.5));

        Distance driveForwardDistance = Feet.of(2);

        Distance offsetFromWallToCenter = Inches.of(16.5);

        // TODO: tune skip and alignment values
        Distance skipPathForDTMTolerance = Inches.of(2.0); // taken out so never skip
        Angle skipPathFromDTMAngleTolerance = Degrees.of(5.0);

        Distance offsetFromWallToCenterDTM = Inches.of(-16.5);

        Pose2d dtmAlignTolerance =
                new Pose2d(Inches.of(1.5), Inches.of(0.8), Rotation2d.fromDegrees(4.0));
    }

    private Command m_autonomousCommand;

    private AutonConfig config;

    public Auton(AutonConfig config) {
        this.config = config;
    }

    // TODO: add visualizer for selected autonomous
    // TODO: add visualizer for dtm

    // TODO: test and tune gui paths

    enum ReefBranch {
        // spotless:off
        A, B, C, D, E, F, G, H, I, J, K, L;
        //spotless:on

        ReefSide reefSide;

        @Getter
        Tusks.Side tusksSide = name().charAt(0) % 2 == 0 ? Tusks.Side.Right : Tusks.Side.Left;

        ReefSide getReefSide() {
            if (reefSide == null) {
                reefSide =
                        Arrays.stream(ReefSide.values())
                                .filter(rs -> rs.name().contains(this.name()))
                                .findFirst()
                                .orElse(null);
            }
            return reefSide;
        }

        void addToChooser(SendableChooser<ReefBranch> chooser, SideOfField sideOfField) {
            var reefSideOfField = getReefSide().getSideOfField();
            if (reefSideOfField.isEmpty() || reefSideOfField.get().equals(sideOfField)) {
                chooser.addOption(name(), this);
            }
        }
    }

    enum ReefSide {
        // spotless:off
        AB, CD, EF, GH, IJ, KL;
        // spotless:on

        @Getter
        final PathPlannerPath fromStartLeft, fromStartRight, fromLeft, fromRight, toLeft, toRight;

        ReefSide() {
            fromStartLeft = loadPath("_FROM_START_LEFT", "_FROM_START");
            fromStartRight = loadPath("_FROM_START_RIGHT", "_FROM_START");
            toLeft = loadPath("_TO_LEFT");
            fromLeft = loadPath("_FROM_LEFT");
            toRight = loadPath("_TO_RIGHT");
            fromRight = loadPath("_FROM_RIGHT");
        }

        PathPlannerPath loadPath(String postFix, String backupPostFix) {
            PathPlannerPath path = loadPath(postFix);

            if (path != null) return path;

            return loadPath(backupPostFix);
        }

        PathPlannerPath loadPath(String postfix) {
            try {
                return PathPlannerPath.fromPathFile(name() + postfix);
            } catch (Exception e) {
            }

            return null;
        }

        Optional<SideOfField> getSideOfField() {
            switch (this) {
                case AB, GH:
                    return Optional.empty();
                case CD, EF:
                    return Optional.of(SideOfField.Right);
                case IJ, KL:
                    return Optional.of(SideOfField.Left);
            }
            throw new RuntimeException();
        }
    }

    public void init() {
        m_autonomousCommand = getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @NoArgsConstructor
    public abstract class AutonStep {
        public abstract PathPlannerPath getPath();

        public Command followPath() {
            return AutoBuilder.followPath(getPath());
        }

        public Pose2d getGoalPose() {
            var poses = getPath().getPathPoses();
            return poses.get(poses.size() - 1);
        }

        public Command alignWithGoalPose() {
            return swerve.driveToPose(getGoalPose());
        }

        public abstract boolean isStepComplete();

        public Command completeStep() {
            return Commands.deadline(
                    followPath()
                            .asProxy()
                            .andThen(
                                    Commands.parallel(
                                            alignWithGoalPose()
                                                    .asProxy()
                                                    .until(this::isStepComplete)),
                                    new WaitUntilCommand(this::isStepComplete)),
                    Commands.startEnd(
                            () -> setCurrentStep(Optional.of(this)),
                            () -> setCurrentStep(Optional.empty())),
                    Commands.startEnd(
                            () -> RobotStates.setDrivingAutonomously(true),
                            () -> RobotStates.setDrivingAutonomously(false)));
        }

        public boolean isLevel(int level) {
            if (this instanceof DeployStep) {
                return ((DeployStep) this).getLevel() == level;
            }
            return false;
        }

        public boolean isTusksSide(Tusks.Side side) {
            if (this instanceof PickupStep) {
                return ((PickupStep) this).getTusksSide().equals(side);
            }
            return false;
        }

        public static Command stepsToCommand(List<AutonStep> steps) {
            var command = Commands.runOnce(() -> Robot.getTusks().getState().setHoldingCoral(true));

            for (var stepCommand :
                    steps.stream().map(AutonStep::completeStep).map(ProxyCommand::new).toList()) {
                command = command.andThen(stepCommand);
            }

            return command;
        }
    }

    @RequiredArgsConstructor
    @Getter
    public class DeployStep extends AutonStep {
        final boolean start;
        final int level;
        final ReefBranch branch;

        @Override
        public PathPlannerPath getPath() {
            var reefSide = branch.getReefSide();
            if (isLeftSideOfFieldSelected()) {
                if (start) return reefSide.getFromStartLeft();
                else return reefSide.getFromLeft();
            } else {
                if (start) return reefSide.getFromStartRight();
                else return reefSide.getFromRight();
            }
        }

        @Override
        public boolean isStepComplete() {
            return !tusks.getState().isHoldingCoral();
        }

        @Override
        public Command alignWithGoalPose() {
            return alignLeftRightOnWall()
                    .asProxy()
                    .until(() -> RobotStates.AlignedWithReef.getAsBoolean());
        }
    }

    @RequiredArgsConstructor
    public class PickupStep extends AutonStep {
        final ReefBranch prevBranch, branch;

        @Override
        public PathPlannerPath getPath() {
            var reefSide = prevBranch.getReefSide();
            if (isLeftSideOfFieldSelected()) return reefSide.getToLeft();
            else return reefSide.getToRight();
        }

        @Override
        public boolean isStepComplete() {
            return tusks.getState().isHoldingCoral();
        }

        public Tusks.Side getTusksSide() {
            return branch.getTusksSide();
        }

        @Override
        public Command alignWithGoalPose() {
            return pushForwardAgainstWall().asProxy().withTimeout(0.5);
        }
    }

    Command getAutonomousCommand() {
        lockChoosers();

        List<AutonStep> steps = new ArrayList<>();

        var firstBranch = getFirstBranch();

        if (firstBranch != null) {

            steps.add(new DeployStep(true, getFirstBranchLevel(), firstBranch));

            var secondBranch = getSecondBranch();

            if (secondBranch != null) {

                steps.add(new PickupStep(firstBranch, secondBranch));
                steps.add(new DeployStep(false, getSecondBranchLevel(), secondBranch));

                var thirdBranch = getThirdBranch();

                if (thirdBranch != null) {
                    steps.add(new PickupStep(secondBranch, thirdBranch));
                    steps.add(new DeployStep(false, getThirdBranchLevel(), thirdBranch));
                }
            }
        }

        unlockChoosers();

        if (steps.size() == 0) return driveForward(config.getDriveForwardDistance());

        return AutonStep.stepsToCommand(steps);
    }

    // Drive forward
    public Command driveForward(Distance distance) {

        return Commands.deferredProxy(
                        () -> {
                            var goalPose = calculatePoseXDistanceAhead(distance);

                            var path =
                                    new PathPlannerPath(
                                            PathPlannerPath.waypointsFromPoses(getPose(), goalPose),
                                            config.pathConstraints,
                                            null,
                                            new GoalEndState(0.0, goalPose.getRotation()));

                            if (path.getAllPathPoints().size() < 5) return Commands.none();

                            path.preventFlipping = true;

                            return AutoBuilder.followPath(path).withName("Auton.Drive Forward");
                        })
                .withName("Auton.Drive Forward Proxy");
    }

    public static Pose2d calculatePoseXDistanceAhead(Distance x) {
        return getPose().plus(new Transform2d(x, Meters.zero(), new Rotation2d()));
    }

    public static class PlaceOption {
        @Getter private ReefSide reefSide;
    }

    public static class PickupOption {
        @Getter private Tusks.Side tusksSide;
    }

    private Command pushForwardAgainstWall() {
        return swerve.driveAgainstWallAlign(
                () -> new Transform2d(Inches.of(2.0), Inches.zero(), new Rotation2d()),
                new Pose2d());
    }

    // DTM
    public Command dtmToHumanPlayerStation() {
        var command =
                Commands.deadline(
                        followPathToAprilTagID(Auton::getClosestHumanPlayerStationID)
                                .andThen(pushForwardAgainstWall().asProxy().withTimeout(0.5)),
                        Commands.startEnd(
                                () -> RobotStates.setDrivingAutonomously(true),
                                () -> RobotStates.setDrivingAutonomously(false)));

        command.setName("DTM TO PICKUP STATION");

        return command;
    }

    public Command dtmToReef() {
        var command =
                Commands.deadline(
                        followPathToAprilTagID(Auton::getClosestReefID)
                                .andThen(
                                        alignLeftRightOnWall()
                                                .asProxy()
                                                .until(
                                                        () ->
                                                                RobotStates.AlignedWithReef
                                                                        .getAsBoolean())),
                        Commands.startEnd(
                                () -> RobotStates.setDrivingAutonomously(true),
                                () -> RobotStates.setDrivingAutonomously(false)));

        command.setName("DTM TO REEF");

        return command;
    }

    public Optional<Pose2d> getDTMToReefGoal() {
        return getClosestReefID().flatMap(this::findGoalPoseInFrontOfTag);
    }

    // Path Planning Helpers
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

        return Robot.getSwerve().getAdditionalState().isPushedUpOnWall()
                && getBumperToReefAlignment().isPresent()
                && bumperToReef.getMeasureY().abs(Meters) < tolerance.getY() * 1.1
                && bumperToReef.getRotation().getMeasure().abs(Degrees)
                        <= tolerance.getRotation().getMeasure().in(Degrees) * 1.1;
    }

    public Transform2d getAlignReefFinalTransform() {
        return getBumperToReefAlignment()
                .orElseGet(
                        () ->
                                getDTMToReefGoal()
                                        .map(
                                                (reefGoalPose) ->
                                                        new Transform2d(getPose(), reefGoalPose))
                                        .orElse(new Transform2d()));
    }

    private Command alignLeftRightOnWall() {
        return swerve.driveAgainstWallAlign(
                        this::getAlignReefFinalTransform, config.getDtmAlignTolerance())
                .asProxy()
                .withTimeout(0.5)
                .andThen(
                        Commands.deferredProxy(
                                () -> {
                                    if (!isBumperToReefAligned()) return alignLeftRightOnWall();
                                    return Commands.none();
                                }));
    }

    private static Rotation2d calculateDirectionToStartDrivingIn(Pose2d goalPose) {
        return swerve.getRobotDriveDirection().orElse(calculateDirectionFromCurrentPose(goalPose));
    }

    private static Rotation2d calculateDirectionFromCurrentPose(Pose2d goalPose) {
        return goalPose.getTranslation().minus(getPose().getTranslation()).getAngle();
    }

    private static Pose2d getPose() {
        return swerve.getState().Pose;
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
                        p ->
                                p.toPose2d()
                                        .plus(
                                                new Transform2d(
                                                        config.getOffsetFromWallToCenter(),
                                                        Meters.zero(),
                                                        Rotation2d.fromDegrees(180))));
    }

    private static CachedValue<Optional<Integer>> cachedClosestHumanPlayerStation =
            new CachedValue<>(Auton::updateClosestHumanPlayerStationID);
    private static CachedValue<Optional<Integer>> cachedClosestReef =
            new CachedValue<>(Auton::updateClosestReefID);

    private static Optional<Integer> updateClosestHumanPlayerStationID() {
        if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1))
                || !isRobotOnOurSide(getPose())) return Optional.empty();

        return SideOfField.getCurrentSide(getPose()).flatMap(SideOfField::getPickUpID);
    }

    public static Optional<Integer> getClosestHumanPlayerStationID() {
        return cachedClosestHumanPlayerStation.get();
    }

    private static Optional<Integer> updateClosestReefID() {
        if (!isRobotOnOurSide(getPose()) || getAllianceReefTags().isEmpty())
            return Optional.empty();

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

    public static Optional<Integer> getClosestReefID() {
        return cachedClosestReef.get();
    }
}
