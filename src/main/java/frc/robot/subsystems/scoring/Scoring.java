package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.HocSubsystem;
import frc.hocLib.Logging;
import frc.reefscape.FieldAndTags2025.ReefLevel;
import frc.robot.Robot;
import frc.robot.RobotStates;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class Scoring extends HocSubsystem {

    @Getter @Setter boolean crossedLine;

    public Scoring() {
        super(new Config("Scoring"));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void periodic() {
        Logging.log(
                "Scoring/CoralGamePieces",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logging.log(
                "Scoring/AlgaeGamePieces",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    void checkForScoredCoral() {
        if (Robot.getForbar().getState().isHoldingCoral()) {

            Robot.getForbar()
                    .getState()
                    .getValidScoringLocation()
                    .ifPresent(
                            validLocation -> {
                                if (RobotBase.isSimulation())
                                    Robot.getForbar().getIntakeSim().obtainGamePieceFromIntake();

                                var bottomOfCoral =
                                        Robot.getForbar()
                                                .getState()
                                                .getComponentOffsets()
                                                .getBottomOfCoral();

                                SimulatedArena.getInstance()
                                        .addGamePieceProjectile(
                                                new ReefscapeCoralOnFly(
                                                        // Obtain robot position
                                                        // from drive simulation
                                                        Robot.getSwerve()
                                                                .getPose()
                                                                .getTranslation(),
                                                        // The scoring mechanism
                                                        // is installed at
                                                        // (0.46, 0) (meters) on
                                                        // the
                                                        // robot
                                                        bottomOfCoral
                                                                .getTranslation()
                                                                .toTranslation2d(),
                                                        // Obtain robot speed
                                                        // from drive simulation
                                                        Robot.getSwerve().getFieldRelativeSpeeds(),
                                                        // Obtain robot facing
                                                        // from drive simulation
                                                        Robot.getSwerve().getPose().getRotation(),
                                                        // The height at which
                                                        // the coral is ejected
                                                        bottomOfCoral.getMeasureZ(),
                                                        // The initial speed of
                                                        // the coral
                                                        MetersPerSecond.of(0),
                                                        // The coral is ejected
                                                        // at a 35-degree slope
                                                        validLocation.getKey().equals(ReefLevel.L4)
                                                                ? Degrees.of(-90)
                                                                : Degrees.of(-35)));
                            });
        }
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
