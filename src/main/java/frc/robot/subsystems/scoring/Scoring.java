package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.HocSubsystem;
import frc.hocLib.Logging;
import frc.hocLib.util.GeomUtil;
import frc.hocLib.util.ImpactDetector;
import frc.reefscape.FieldAndTags2025;
import frc.reefscape.FieldAndTags2025.ReefLevel;
import frc.robot.Robot;
import frc.robot.RobotStates;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class Scoring extends HocSubsystem {

    @Getter @Setter boolean crossedLine;

    @Getter List<Pose3d> reefAlgae = new ArrayList<>();

    public Scoring() {
        super(new Config("Scoring"));

        resetFieldForAuto();
    }

    @Override
    public void simulationPeriodic() {
        Logging.log(
                "Scoring/CoralGamePieces",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logging.log(
                "Scoring/AlgaeGamePieces",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

        Logging.log("Scoring/Reef Algae", reefAlgae.toArray(new Pose3d[reefAlgae.size()]));
    }

    public void resetFieldForAuto() {
        if (RobotBase.isSimulation()) SimulatedArena.getInstance().resetFieldForAuto();

        var centerY = FieldAndTags2025.FIELD_WIDTH.div(2);
        var centerX = FieldAndTags2025.FIELD_LENGTH.div(2);

        var abXFromCenterX = Millimeters.of(4964.12);
        var cdklFromCenterX = Millimeters.of(4624.48);
        var efijFromCenterX = Millimeters.of(3945.1);
        var ghFromCenterX = Millimeters.of(3605.45);

        var cdefijklFromCenterY = Millimeters.of(588.288);

        var algaeL3Z = Millimeters.of(1313.18);
        var algaeL2Z = Millimeters.of(909.32);

        var abAlgae =
                new Pose3d(centerX.minus(abXFromCenterX), centerY, algaeL3Z, Rotation3d.kZero);

        var cdAlgae =
                new Pose3d(
                        centerX.minus(cdklFromCenterX),
                        centerY.minus(cdefijklFromCenterY),
                        algaeL2Z,
                        Rotation3d.kZero);

        var efAlgae =
                new Pose3d(
                        centerX.minus(efijFromCenterX),
                        centerY.minus(cdefijklFromCenterY),
                        algaeL3Z,
                        Rotation3d.kZero);

        var ghAlgae = new Pose3d(centerX.minus(ghFromCenterX), centerY, algaeL2Z, Rotation3d.kZero);

        var ijAlgae =
                new Pose3d(
                        centerX.minus(efijFromCenterX),
                        centerY.plus(cdefijklFromCenterY),
                        algaeL3Z,
                        Rotation3d.kZero);

        var klAlgae =
                new Pose3d(
                        centerX.minus(cdklFromCenterX),
                        centerY.plus(cdefijklFromCenterY),
                        algaeL2Z,
                        Rotation3d.kZero);

        reefAlgae.clear();
        reefAlgae.addAll(List.of(abAlgae, cdAlgae, efAlgae, ghAlgae, ijAlgae, klAlgae));
    }

    void launchCoral(Angle launchAngle) {
        if (RobotBase.isSimulation()) Robot.getForbar().getIntakeSim().obtainGamePieceFromIntake();

        var bottomOfCoral = Robot.getForbar().getState().getComponentOffsets().getBottomOfCoral();

        if (RobotBase.isSimulation())
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                    // Obtain robot position
                                    // from drive simulation
                                    Robot.getSwerve().getPose().getTranslation(),
                                    // The scoring mechanism
                                    // is installed at
                                    // (0.46, 0) (meters) on
                                    // the
                                    // robot
                                    bottomOfCoral.getTranslation().toTranslation2d(),
                                    // Obtain robot speed
                                    // from drive simulation
                                    RobotStates.GoToL1Coral.getAsBoolean()
                                            ? impactDetector.getImpactChassisSpeeds()
                                            : Robot.getSwerve().getFieldRelativeSpeeds(),
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
                                    launchAngle == null
                                            ? bottomOfCoral.getRotation().getMeasureY()
                                            : launchAngle));
    }

    void checkForScoredCoral() {
        if (Robot.getForbar().getState().isHoldingCoral()) {

            if (RobotStates.GoToL1Coral.getAsBoolean()) {
                var impactSpeeds = impactDetector.getImpactChassisSpeeds();
                var impactDirection =
                        new Translation2d(
                                        impactSpeeds.vxMetersPerSecond,
                                        impactSpeeds.vyMetersPerSecond)
                                .getAngle()
                                .getMeasure();
                if (impactDirection
                        .minus(Robot.getSwerve().getPose().getRotation().getMeasure())
                        .isNear(Degrees.zero(), Degrees.of(60))) launchCoral(null);
            } else {

                Robot.getForbar()
                        .getState()
                        .getValidScoringLocation()
                        .ifPresent(
                                validLocation -> {
                                    launchCoral(
                                            validLocation.getKey().equals(ReefLevel.L4)
                                                    ? Degrees.of(-90)
                                                    : Degrees.of(-35));
                                });
            }
        }
    }

    void checkForRemoveAlgae() {
        var lengthOfAlgaeArm = Inches.of(11);
        var radiusOfAlgae = Units.inchesToMeters(8.0);

        var algaeWheelPosition =
                Robot.getJaws()
                        .getState()
                        .getComponentOffset()
                        .transformBy(GeomUtil.toTransform3d(lengthOfAlgaeArm, 0.0, 0.0))
                        .getTranslation();
        var algaeWheelPositionOnField =
                new Pose3d(Robot.getSwerve().getPose())
                        .transformBy(GeomUtil.toTransform3d(algaeWheelPosition))
                        .getTranslation();
        for (int i = 0; i < reefAlgae.size(); i++) {
            var algaePosition = reefAlgae.get(i).getTranslation();
            if (algaePosition.minus(algaeWheelPositionOnField).getNorm() > radiusOfAlgae) continue;

            reefAlgae.remove(i);

            if (RobotBase.isSimulation())
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                                new ReefscapeAlgaeOnFly(
                                        algaePosition.toTranslation2d(),
                                        new Translation2d(),
                                        new ChassisSpeeds(),
                                        algaePosition
                                                .toTranslation2d()
                                                .minus(FieldAndTags2025.CenterOfBlueReef)
                                                .getAngle(),
                                        algaePosition.getMeasureZ(),
                                        MetersPerSecond.of(1),
                                        Degrees.of(0)));
        }
    }

    static ImpactDetector impactDetector = new ImpactDetector(MetersPerSecondPerSecond.of(3.0));

    static final Trigger ImpactDetected =
            new Trigger(() -> impactDetector.update(Robot.getSwerve().getFieldRelativeSpeeds()));

    @Override
    public void setupBindings() {
        RobotStates.GoToL2Coral.or(RobotStates.GoToL3Coral, RobotStates.GoToL4Coral)
                .not()
                .onTrue(Commands.runOnce(this::checkForScoredCoral));

        ImpactDetected.and(RobotStates.GoToL1Coral)
                .onTrue(Commands.runOnce(this::checkForScoredCoral));

        new Trigger(() -> Robot.getJaws().getState().isIn())
                .not()
                .whileTrue(Commands.run(this::checkForRemoveAlgae));
    }

    @Override
    public void setupDefaultCommand() {}
}
