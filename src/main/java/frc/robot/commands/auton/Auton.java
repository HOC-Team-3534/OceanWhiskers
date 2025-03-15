package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Auton {

    static final EventTrigger autonDeploy = new EventTrigger("Deploy");
    static final EventTrigger autonPickup = new EventTrigger("Pickup");

    public static Trigger isLevel(int level) {
        return new Trigger(
                        () ->
                                AutonStep.getCurrentStep()
                                        .map(step -> step.isLevel(level))
                                        .orElse(false))
                .and(autonDeploy);
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
    }

    private Command m_autonomousCommand;

    private AutonConfig config;

    public Auton(AutonConfig config) {
        this.config = config;
    }

    // TODO: add visualizer for selected autonomous
    // TODO: add visualizer for dtm

    // TODO: test and tune gui paths

    public void init() {
        loadAutonomousCommand();
        if (RobotBase.isSimulation()) resetPoseToStartOfPath();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    AutonChoosers.Choices choices = null;
    List<AutonStep> steps = new ArrayList<>();

    public void loadAutonomousCommand() {
        var newChoices = AutonChoosers.Choices.load();

        if (newChoices == null || (choices != null && newChoices.equals(choices))) return;

        choices = newChoices;

        if (choices.getFirstBranch() != null) {

            steps.add(
                    new DeployStep(
                            true,
                            choices.getFirstBranchLevel(),
                            choices.getFirstBranch(),
                            choices.getSideOfField()));

            if (choices.getSecondBranch() != null) {

                steps.add(new PickupStep(choices.getFirstBranch(), choices.getSideOfField()));
                steps.add(
                        new DeployStep(
                                false,
                                choices.getSecondBranchLevel(),
                                choices.getSecondBranch(),
                                choices.getSideOfField()));

                if (choices.getThirdBranch() != null) {
                    steps.add(new PickupStep(choices.getSecondBranch(), choices.getSideOfField()));
                    steps.add(
                            new DeployStep(
                                    false,
                                    choices.getThirdBranchLevel(),
                                    choices.getThirdBranch(),
                                    choices.getSideOfField()));
                }
            }
        }

        if (steps.size() == 0) {
            m_autonomousCommand = driveForward(config.getDriveForwardDistance());
            return;
        }

        m_autonomousCommand = AutonStep.stepsToCommand(steps);
    }

    void resetPoseToStartOfPath() {
        if (steps.size() > 0) {
            var firstPath = steps.get(0).getPath();
            firstPath
                    .getStartingHolonomicPose()
                    .ifPresent((startingPose) -> Robot.getSwerve().resetPose(startingPose));
        }
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

    private static Pose2d getPose() {
        return Robot.getSwerve().getPose();
    }
}
