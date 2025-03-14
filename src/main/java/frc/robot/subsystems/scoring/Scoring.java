package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.HocSubsystem;
import frc.hocLib.util.GeomUtil;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.ReefLevel;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.commands.auton.DTM;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import lombok.Getter;
import lombok.Setter;

public class Scoring extends HocSubsystem {

    @Getter @Setter boolean crossedLine;

    @Getter Set<Pair<ReefBranch, ReefLevel>> scoredCoral = new HashSet<>();

    @Getter List<Pose3d> coralLocations = new ArrayList<>();

    @Getter Pose3d[] coralLocationsArray = new Pose3d[0];

    public Scoring() {
        super(new Config("Scoring"));
    }

    void checkForScoredCoral() {
        DTM.getClosestReefBranch()
                .ifPresent(
                        (branch) -> {
                            var bottomOfCoral =
                                    new Pose3d(Robot.getSwerve().getPose())
                                            .transformBy(
                                                    GeomUtil.toTransform3d(
                                                            Robot.getForbar()
                                                                    .getState()
                                                                    .getComponentOffsets()
                                                                    .getBottomOfCoral()));

                            Function<Pose3d, Distance> distanceFromCoral =
                                    (location) ->
                                            Meters.of(
                                                    location.getTranslation()
                                                            .minus(bottomOfCoral.getTranslation())
                                                            .getNorm());

                            branch.getScoringLocations().entrySet().stream()
                                    .filter(
                                            (location) ->
                                                    distanceFromCoral
                                                            .apply(location.getValue())
                                                            .lt(
                                                                    location.getKey()
                                                                                    .equals(
                                                                                            ReefLevel
                                                                                                    .L4)
                                                                            ? Inches.of(6.0)
                                                                            : Inches.of(4.0)))
                                    .findFirst()
                                    .ifPresent(
                                            validLocation -> {
                                                var size = scoredCoral.size();
                                                scoredCoral.add(
                                                        new Pair<ReefBranch, ReefLevel>(
                                                                branch, validLocation.getKey()));
                                                if (scoredCoral.size() != size) {
                                                    coralLocations.add(
                                                            branch.getScoredCoral(
                                                                    validLocation.getKey()));
                                                    coralLocationsArray =
                                                            coralLocations.toArray(
                                                                    new Pose3d
                                                                            [coralLocations
                                                                                    .size()]);
                                                }
                                            });
                        });
    }

    @Override
    public void setupBindings() {
        RobotStates.GoToL2Coral.or(RobotStates.GoToL3Coral, RobotStates.GoToL4Coral)
                .not()
                .onTrue(Commands.runOnce(this::checkForScoredCoral));
    }

    @Override
    public void setupDefaultCommand() {}
}
