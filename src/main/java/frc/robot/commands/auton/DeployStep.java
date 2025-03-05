package frc.robot.commands.auton;

import static frc.robot.commands.auton.AutonChoosers.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
@Getter
public class DeployStep extends AutonStep {
    final boolean start;
    final int level;

    final ReefBranch branch;

    @Getter private Timer deployTimeout = new Timer();

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
        return !Robot.getTusks().getState().isHoldingCoral();
    }

    @Override
    public Command alignWithGoalPose() {
        return Commands.runOnce(() -> deployTimeout.restart())
                .andThen(Robot.getDtm().alignLeftRightOnReefWall().asProxy());
    }
}
