package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.hocLib.swerve.DriveSpeedsConsumer;
import frc.hocLib.swerve.FieldRelativeSpeedsSupplier;
import frc.hocLib.swerve.MaxKinematicsSupplier;
import frc.hocLib.swerve.RobotPoseSupplier;
import frc.hocLib.util.LoggedTunableNumber;
import java.util.function.BiConsumer;
import lombok.Setter;

public class FollowPathThenDriveToPose<
                T extends
                        Subsystem & RobotPoseSupplier & DriveSpeedsConsumer
                                & FieldRelativeSpeedsSupplier & MaxKinematicsSupplier>
        extends Command {
    private final FollowPathCommand followPathCommand;
    private final DriveToPose<T> driveToPoseCommand;

    @Setter private Pair<ChassisSpeeds, DriveFeedforwards> latestFollowPathCommandOutput;
    @Setter private ChassisSpeeds latestDriveToPoseOutput;

    private final BiConsumer<ChassisSpeeds, DriveFeedforwards> mainOutput;

    private static final LoggedTunableNumber secondsToStartMerge =
            new LoggedTunableNumber("FollowPathThenDriveToPose/SecondsToStartMerge");
    private static final LoggedTunableNumber mergeDuration =
            new LoggedTunableNumber("FollowPathThenDriveToPose/MergeDuration");

    private boolean driveToPoseInitialized;

    public FollowPathThenDriveToPose(
            FollowPathCommand followPathCommand, DriveToPose<T> driveToPoseCommand) {

        this.mainOutput = followPathCommand.getOutput();

        this.followPathCommand =
                followPathCommand.withOutput(
                        (speeds, ff) ->
                                setLatestFollowPathCommandOutput(
                                        new Pair<ChassisSpeeds, DriveFeedforwards>(speeds, ff)));
        this.driveToPoseCommand = driveToPoseCommand.withOutput(this::setLatestDriveToPoseOutput);

        secondsToStartMerge.initDefault(0.75);
        mergeDuration.initDefault(0.25);
    }

    @Override
    public void initialize() {
        followPathCommand.initialize();
        driveToPoseInitialized = false;
    }

    @Override
    public void execute() {

        var isFinishedFollowingPath = followPathCommand.isFinished();

        if (!isFinishedFollowingPath) followPathCommand.execute();

        var secondsIntoMerge =
                secondsToStartMerge.get() - followPathCommand.timeUntilDone().in(Seconds);

        var speeds =
                isFinishedFollowingPath || secondsIntoMerge > mergeDuration.get()
                        ? new ChassisSpeeds()
                        : latestFollowPathCommandOutput.getFirst();
        var feedforwards =
                isFinishedFollowingPath || secondsIntoMerge > mergeDuration.get()
                        ? DriveFeedforwards.zeros(4)
                        : latestFollowPathCommandOutput.getSecond();

        if (secondsIntoMerge >= 0) {
            if (!driveToPoseInitialized) {
                driveToPoseCommand.initialize();
                driveToPoseInitialized = true;
            }

            var driveToPoseRatio = Math.min(secondsIntoMerge / mergeDuration.get(), 1);
            var followPathRatio = 1 - driveToPoseRatio;

            driveToPoseCommand.execute();

            speeds =
                    speeds.times(followPathRatio)
                            .plus(latestDriveToPoseOutput.times(driveToPoseRatio));
        }

        mainOutput.accept(speeds, feedforwards);
    }
}
