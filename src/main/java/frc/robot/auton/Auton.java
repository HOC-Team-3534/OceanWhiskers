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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.Robot;
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
        return new Trigger(() -> getCurrentStep().map(step -> step.isLevel(1)).orElse(false));
    }

    public static Trigger isTusksSide(Tusks.Side side) {
        return new Trigger(
                () -> getCurrentStep().map(step -> step.isTusksSide(side)).orElse(false));
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

        Distance offsetFromWallToCenter = Inches.of(16);

        // TODO: tune skip and alignment values
        Distance skipPathForDTMTolerance = Inches.of(2.0); // taken out so never skip
        Angle skipPathFromDTMAngleTolerance = Degrees.of(5.0);

        Distance offsetFromWallToCenterDTM = Inches.of(15.2);

        Distance alignFwdTolerance = Inches.of(0.15);
        Distance alignLeftRightTolerance = Inches.of(0.6);
        Angle alignAngleTolerance = Degrees.of(1.0);
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
            return swerve.preciseAlignment(getGoalPose());
        }

        public abstract boolean isStepComplete();

        public Command completeStep() {
            return Commands.deadline(
                    followPath()
                            .andThen(alignWithGoalPose())
                            .andThen(new WaitUntilCommand(this::isStepComplete)),
                    Commands.startEnd(
                            () -> setCurrentStep(Optional.of(this)),
                            () -> setCurrentStep(Optional.empty())));
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
            return Commands.parallel(
                    (Command[]) steps.stream().map(AutonStep::completeStep).toArray());
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
            return alignLeftRightOnWall();
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
            return driveForward(Inches.of(5.0));
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
    // DTM
    public Command dtmToHumanPlayerStation() {
        return followPathToAprilTagID(Auton::findClosestHumanPlayerStationID)
                .andThen(driveForward(Inches.of(5.0)).asProxy());
    }

    public Command dtmToReef() {
        return followPathToAprilTagID(Auton::findClosestReefID)
                .andThen(alignLeftRightOnWall().asProxy());
    }

    public Optional<Pose2d> getDTMToReefGoal() {
        return findClosestReefID().flatMap(this::findGoalPoseInFrontOfTag);
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

                                        path.preventFlipping = true;

                                        return AutoBuilder.followPath(path);
                                    })
                            .orElse(Commands.none());
                });
    }

    public Optional<Distance> getDistanceToAlignFwd() {
        return Robot.getVisionSystem()
                .getDistanceToAlignFwd()
                .map(
                        distance -> {
                            var fwdError = distance.minus(config.getOffsetFromWallToCenterDTM());

                            if (fwdError.lt(Inches.zero())) return Inches.zero();

                            return fwdError;
                        });
    }

    private Command alignLeftRightOnWall() {
        return swerve.alignLeftRightOnWall(
                this::getDistanceToAlignFwd,
                config.getAlignFwdTolerance(),
                () -> Robot.getVisionSystem().getDistanceToAlignLeftPositive(),
                config.getAlignLeftRightTolerance(),
                () -> Robot.getVisionSystem().getAngleToAlign(),
                config.getAlignAngleTolerance());
    }

    private static Rotation2d calculateDirectionToStartDrivingIn(Pose2d goalPose) {
        return swerve.getRobotDriveDirection().orElse(calculateDirectionFromCurrentPose(goalPose));
    }

    private static Rotation2d calculateDirectionFromCurrentPose(Pose2d goalPose) {
        return goalPose.getTranslation().minus(getPose().getTranslation()).getAngle();
    }

    private static boolean isRobotOnOurSide() {
        var values = getAllianceValues();
        if (values.isEmpty()) return false;
        return values.get()
                .getDistanceFromAllianceWall(getPose())
                .lte(FIELD_LENGTH.div(2).plus(Feet.of(1)));
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

    private static Optional<Integer> findClosestHumanPlayerStationID() {
        if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1)) || !isRobotOnOurSide())
            return Optional.empty();

        return SideOfField.getCurrentSide(getPose()).flatMap(SideOfField::getPickUpID);
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
