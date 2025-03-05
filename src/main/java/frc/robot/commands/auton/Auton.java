package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.commands.auton.AutonChoosers.getFirstBranch;
import static frc.robot.commands.auton.AutonChoosers.getFirstBranchLevel;
import static frc.robot.commands.auton.AutonChoosers.getSecondBranch;
import static frc.robot.commands.auton.AutonChoosers.getSecondBranchLevel;
import static frc.robot.commands.auton.AutonChoosers.getThirdBranch;
import static frc.robot.commands.auton.AutonChoosers.getThirdBranchLevel;
import static frc.robot.commands.auton.AutonChoosers.lockChoosers;
import static frc.robot.commands.auton.AutonChoosers.unlockChoosers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.tusks.Tusks;
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

    public static Trigger isTusksSide(Tusks.Side side) {
        return new Trigger(
                        () ->
                                AutonStep.getCurrentStep()
                                        .map(step -> step.isTusksSide(side))
                                        .orElse(false))
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
        m_autonomousCommand = getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
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

    private static Pose2d getPose() {
        return Robot.getSwerve().getState().Pose;
    }
}
