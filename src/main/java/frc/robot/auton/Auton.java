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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldAndTags2025.ReefSide;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.swerve.Swerve;
import frc.robot.tusks.Tusks;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Auton {
    static final EventTrigger autonDeploy1 = new EventTrigger("Deploy1");
    static final EventTrigger autonDeploy2 = new EventTrigger("Deploy2");
    static final EventTrigger autonDeploy3 = new EventTrigger("Deploy3");

    static final EventTrigger autonPickup2 = new EventTrigger("Pickup2");
    static final EventTrigger autonPickup3 = new EventTrigger("Pickup3");

    public Trigger autonL4 = Trigger.kFalse;
    public Trigger autonL3 = Trigger.kFalse;
    public Trigger autonL2 = Trigger.kFalse;
    public Trigger autonL1 = Trigger.kFalse;

    public Trigger autonPickupLeft = Trigger.kFalse;
    public Trigger autonPickupRight = Trigger.kFalse;

    private static final Swerve swerve = Robot.getSwerve();

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

        double percentSlowEndOfPath = 0.2;

        Distance offsetFromWallToCenter = Inches.of(17.0); // TODO: measure and change
    }

    private Command m_autonomousCommand;

    private final SendableChooser<Command> autonChooser;
    private final SendableChooser<Integer> deploy1Level, deploy2Level, deploy3Level;
    private final SendableChooser<Tusks.Side> pickup2Side, pickup3Side;

    private AutonConfig config;

    public Auton(AutonConfig config) {
        this.config = config;

        autonChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auton/Auton", autonChooser);

        deploy1Level = buildLevelChooser();
        deploy2Level = buildLevelChooser();
        deploy3Level = buildLevelChooser();

        SmartDashboard.putData("Auton/Deploy 1 Level", deploy1Level);
        SmartDashboard.putData("Auton/Deploy 2 Level", deploy2Level);
        SmartDashboard.putData("Auton/Deploy 3 Level", deploy3Level);

        pickup2Side = buildPickupSideChooser();
        pickup3Side = buildPickupSideChooser();

        SmartDashboard.putData("Auton/Pickup 2 Side", pickup2Side);
        SmartDashboard.putData("Auton/Pickup 3 Side", pickup3Side);

        NamedCommands.registerCommand(
                "waitUntilCoralDeployed", new WaitUntilCommand(RobotStates.HoldingCoral.not()));
        NamedCommands.registerCommand(
                "waitUntilCoralPickedUp", new WaitUntilCommand(RobotStates.HoldingCoral));
    }

    private SendableChooser<Integer> buildLevelChooser() {
        var chooser = new SendableChooser<Integer>();

        chooser.setDefaultOption("4", 4);
        chooser.addOption("3", 3);
        chooser.addOption("2", 2);
        chooser.addOption("1", 1);

        return chooser;
    }

    private SendableChooser<Tusks.Side> buildPickupSideChooser() {
        var chooser = new SendableChooser<Tusks.Side>();

        chooser.setDefaultOption("Left", Tusks.Side.Left);
        chooser.addOption("Right", Tusks.Side.Right);

        return chooser;
    }

    public void init() {
        m_autonomousCommand = getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    void updateDeployLevel(int level, Trigger trigger) {
        switch (level) {
            case 1:
                autonL1 = autonL1.or(trigger);
                break;
            case 2:
                autonL2 = autonL2.or(trigger);
                break;
            case 3:
                autonL3 = autonL3.or(trigger);
                break;
            case 4:
                autonL4 = autonL4.or(trigger);
                break;
        }
    }

    void updateTusksPickupSide(Tusks.Side side, Trigger trigger) {
        switch (side) {
            case Left:
                autonPickupLeft = autonPickupLeft.or(trigger);
                break;
            case Right:
                autonPickupRight = autonPickupRight.or(trigger);
                break;
        }
    }

    void updateAutonTriggers() {
        autonL1 = Trigger.kFalse;
        autonL2 = Trigger.kFalse;
        autonL3 = Trigger.kFalse;
        autonL4 = Trigger.kFalse;

        autonPickupLeft = Trigger.kFalse;
        autonPickupRight = Trigger.kFalse;

        updateDeployLevel(deploy1Level.getSelected(), autonDeploy1);
        updateDeployLevel(deploy2Level.getSelected(), autonDeploy2);
        updateDeployLevel(deploy3Level.getSelected(), autonDeploy3);

        updateTusksPickupSide(pickup2Side.getSelected(), autonPickup2);
        updateTusksPickupSide(pickup3Side.getSelected(), autonPickup3);
    }

    Command getAutonomousCommand() {
        updateAutonTriggers();
        return autonChooser.getSelected();
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

                    return AutoBuilder.followPath(path);
                });
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
        return followPathToAprilTagID(Auton::findClosestHumanPlayerStationID);
    }

    public Command dtmToReef() {
        return followPathToAprilTagID(Auton::findClosestReefID);
    }

    // Path Planning Helpers
    private Command followPathToAprilTagID(Supplier<Optional<Integer>> tagIdSupplier) {
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
                                        var path =
                                                new PathPlannerPath(
                                                        PathPlannerPath.waypointsFromPoses(
                                                                startPose, goalPose),
                                                        config.pathConstraints,
                                                        null,
                                                        new GoalEndState(
                                                                0.0, goalPose.getRotation()));

                                        path.preventFlipping = true;

                                        return (Command)
                                                AutoBuilder.followPath(path)
                                                        .andThen(swerve.preciseAlignment(goalPose));
                                    })
                            .orElse(Commands.none());
                });
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
