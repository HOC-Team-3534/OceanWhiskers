package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.hocLib.util.Util;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.tusks.Tusks;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@NoArgsConstructor
public abstract class AutonStep {
    @Getter @Setter private static Optional<AutonStep> currentStep = Optional.empty();

    public abstract PathPlannerPath getPath();

    public Command followPath() {
        return AutoBuilder.followPath(getPath());
    }

    public Pose2d getGoalPose() {
        var path = Util.isRedAlliance() ? getPath().flipPath() : getPath();
        var points = path.getAllPathPoints();
        var lastPoint = points.get(points.size() - 1);
        return new Pose2d(lastPoint.position, lastPoint.rotationTarget.rotation());
    }

    public Command alignWithGoalPose() {
        return Robot.getSwerve().driveToPose(getGoalPose());
    }

    public abstract boolean isStepComplete();

    public Command completeStep() {
        return Commands.deadline(
                followPath()
                        .asProxy()
                        .andThen(
                                Commands.parallel(
                                        alignWithGoalPose().asProxy().until(this::isStepComplete)),
                                new WaitUntilCommand(this::isStepComplete)),
                Commands.startEnd(
                        () -> setCurrentStep(Optional.of(this)),
                        () -> setCurrentStep(Optional.empty())),
                Commands.startEnd(
                        () -> RobotStates.setDrivingAutonomously(true),
                        () -> RobotStates.setDrivingAutonomously(false)));
    }

    public boolean isDeployTimedOut() {
        if (this instanceof DeployStep) {
            return ((DeployStep) this).getDeployTimeout().hasElapsed(3.0);
        }
        return false;
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
