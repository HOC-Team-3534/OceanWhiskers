package frc.robot.subsystems.forbar;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonFXMechanism;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.GeomUtil;
import frc.reefscape.FieldAndTags2025.ReefLevel;
import frc.robot.Robot;
import frc.robot.commands.auton.DTM;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Function;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class Forbar extends TalonFXMechanism {
    public static class ForbarConfig extends TalonFXMechanism.Config {
        @Getter private Voltage outVoltage = Volts.of(3.0); // out is positive
        @Getter private Voltage holdOutVoltage = Volts.of(1.25); // out is positive
        @Getter private Voltage inVoltage = Volts.of(-3.0); // in is negative
        @Getter private Power spikeThreshold = outVoltage.times(Amps.of(10.0));

        Pose3d fieldToRobotCAD = new Pose3d(0.0, 0.0, 0.0414, new Rotation3d());

        Pose3d robotToLongPivot =
                fieldToRobotCAD.transformBy(
                        GeomUtil.toTransform3d(new Translation3d(0.17, 0.0, 0.4425)));
        Pose3d robotToShortPivot =
                fieldToRobotCAD.transformBy(
                        GeomUtil.toTransform3d(new Translation3d(0.28375, 0.0, 0.4325)));
        Pose3d robotToCarriage =
                fieldToRobotCAD.transformBy(
                        GeomUtil.toTransform3d(new Translation3d(0.22, 0.0, 0.6525)));

        Translation3d carriageToBottomOfCoral =
                new Translation3d(Inches.of(1.625), Inches.zero(), Inches.of(-2.647));

        Angle shortPivotPitchOffset = Degrees.of(-16.23 - 90.0);
        Angle shortPivotPitchRange = Degrees.of(16.23 + 43.92);

        Distance shortPivotLength = Inches.of(9.0);
        Distance longPivotLength = Inches.of(10.5);
        Distance backPlatePivotSeparation = Inches.of(3.01);

        @Getter private Time timeFromInToOut = Seconds.of(0.25);

        public ForbarConfig() {
            super("Forbar", 16);
            var motorOutputConfigs = new MotorOutputConfigs();

            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

            setMotorOutputConfigs(motorOutputConfigs);
        }
    }

    private ForbarConfig config;

    @Getter private State state = new State();

    private CANrange canRange;
    private CANrangeSimState canRangeSim;

    @Getter private IntakeSimulation intakeSim = null;

    public Forbar(ForbarConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            canRange = new CANrange(19);

            var canRangeConfig = new CANrangeConfiguration();

            canRangeConfig.ProximityParams.ProximityThreshold = Inches.of(4.0).in(Meters);
            canRangeConfig.ProximityParams.ProximityHysteresis = Inches.of(2.0).in(Meters);

            canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

            canRangeConfig.FovParams.FOVCenterX = 0.0;
            canRangeConfig.FovParams.FOVCenterY = 0.0;
            canRangeConfig.FovParams.FOVRangeX = 27.0;
            canRangeConfig.FovParams.FOVRangeY = 27.0;

            canRange.getConfigurator().apply(canRangeConfig);

            canRangeSim = canRange.getSimState();

            canRangeSim.setDistance(Inches.of(24.0));
        }

        if (RobotBase.isSimulation()) {
            SmartDashboard.putBoolean("HoldingCoralOverride", false);

            intakeSim =
                    IntakeSimulation.InTheFrameIntake(
                            "Coral",
                            Robot.getSwerve().getMapleSimSwerveDrivetrain().mapleSimDrive,
                            Inches.of(6.0),
                            IntakeSide.FRONT,
                            1);
        }
    }

    private boolean prevCanRangeDetectedObject;

    @Override
    public void periodic() {
        if (isAttached()) {
            var detectedObject = canRange.getIsDetected().getValue().booleanValue();

            if (detectedObject != prevCanRangeDetectedObject) {
                state.setHoldingCoral(detectedObject);
                prevCanRangeDetectedObject = detectedObject;
            }
        }

        Logging.log("Forbar", this);
    }

    boolean prevHoldingCoralOverride;
    int prevIntakeSimCount;
    Timer simTimeAlignedWithPickup = new Timer();

    @Override
    public void simulationPeriodic() {
        Robot.getDtm()
                .finalGoalPoseInFrontOfClosestLoadingStation()
                .ifPresentOrElse(
                        loadingStationPose -> {
                            var relative =
                                    loadingStationPose.relativeTo(Robot.getSwerve().getPose());

                            if (relative.getMeasureX().isNear(Inches.zero(), Inches.of(2.0))
                                    && relative.getMeasureY().isNear(Inches.zero(), Inches.of(24.0))
                                    && Math.abs(relative.getRotation().getDegrees()) < 3.0) {

                            } else {
                                simTimeAlignedWithPickup.restart();
                            }
                        },
                        simTimeAlignedWithPickup::restart);

        if (simTimeAlignedWithPickup.hasElapsed(0.5) && !state.isHoldingCoral())
            intakeSim.addGamePieceToIntake();

        var holdingCoralOverride = SmartDashboard.getBoolean("HoldingCoralOverride", false);

        if (holdingCoralOverride != prevHoldingCoralOverride) {
            if (holdingCoralOverride) intakeSim.addGamePieceToIntake();
            else intakeSim.obtainGamePieceFromIntake();
            prevHoldingCoralOverride = holdingCoralOverride;
        }

        var intakeSimCount = intakeSim.getGamePiecesAmount();

        if (intakeSimCount != prevIntakeSimCount) {
            if (isAttached()) {
                canRangeSim.setSupplyVoltage(RobotController.getBatteryVoltage());
                canRangeSim.setDistance(intakeSimCount > 0 ? Inches.of(1.5) : Inches.of(12.0));
            } else {
                state.setHoldingCoral(intakeSimCount > 0);
            }
            prevIntakeSimCount = intakeSimCount;
        }
    }

    private boolean isPowerSpikeExceeded() {
        return state.currentPositionTimer.hasElapsed(config.timeFromInToOut.in(Seconds));
    }

    protected Command zeroOrHold() {
        return run(
                () ->
                        setVoltageOut(
                                state.getPosition().equals(Position.Out)
                                        ? config.getHoldOutVoltage()
                                        : Volts.zero()));
    }

    // TODO: validate open and close happen when theya re supposed to and not when elevator is not
    // raised

    protected Command in() {
        return Commands.waitSeconds(0.25)
                .andThen(
                        startRun(
                                        () -> state.setPosition(Position.GoingIn),
                                        () -> setVoltageOut(config.getInVoltage()))
                                .until(this::isPowerSpikeExceeded),
                        runOnce(() -> state.setPosition(Position.In)));
    }

    protected Command out() {
        return startRun(
                        () -> state.setPosition(Position.GoingOut),
                        () -> setVoltageOut(config.getOutVoltage()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.Out)));
    }

    public enum Position {
        In,
        Out,
        GoingOut,
        GoingIn
    }

    @Getter
    @Setter
    public class State {
        boolean holdingCoral;
        Position position = Position.In;
        Pose3d currentOffset = new Pose3d();
        Timer currentPositionTimer = new Timer();
        CachedValue<ForbarComponentOffsets> componentOffsets =
                new CachedValue<>(() -> updateForbarComponentPositions(getPercentOutVsIn()));
        CachedValue<Optional<Entry<ReefLevel, Pose3d>>> validScoringLocation =
                new CachedValue<>(() -> updateValidScoringLocation());

        State() {
            currentPositionTimer.restart();
        }

        public void setPosition(Position position) {
            if (!this.position.equals(position)) {
                currentPositionTimer.restart();
                this.position = position;
            }
        }

        public boolean isOut() {
            return position.equals(Position.Out);
        }

        public boolean isIn() {
            return position.equals(Position.In);
        }

        public ForbarComponentOffsets getComponentOffsets() {
            return componentOffsets.get();
        }

        public Optional<Entry<ReefLevel, Pose3d>> getValidScoringLocation() {
            return validScoringLocation.get();
        }

        public Distance getCANrangeDistance() {
            return isAttached()
                    ? canRange.getDistance().getValue()
                    : Meters.of(Double.POSITIVE_INFINITY);
        }

        public boolean isCANrangeIsDetected() {
            return isAttached() ? canRange.getIsDetected().getValue().booleanValue() : false;
        }

        private double getPercentOutVsIn() {
            return switch (position) {
                case GoingIn:
                    yield 1
                            - Math.min(
                                    currentPositionTimer.get() / config.timeFromInToOut.in(Seconds),
                                    1);
                case GoingOut:
                    yield Math.min(
                            currentPositionTimer.get() / config.timeFromInToOut.in(Seconds), 1);
                case In:
                    yield 0.0;
                case Out:
                    yield 1.0;
            };
        }

        Optional<Entry<ReefLevel, Pose3d>> updateValidScoringLocation() {
            return DTM.getClosestReefBranch()
                    .flatMap(
                            branch -> {
                                var bottomOfCoral =
                                        new Pose3d(Robot.getSwerve().getPose())
                                                .transformBy(
                                                        GeomUtil.toTransform3d(
                                                                getComponentOffsets()
                                                                        .bottomOfCoral));

                                Function<Pose3d, Distance> xyDistanceFromCoral =
                                        (location) ->
                                                Meters.of(
                                                        location.getTranslation()
                                                                .toTranslation2d()
                                                                .minus(
                                                                        bottomOfCoral
                                                                                .getTranslation()
                                                                                .toTranslation2d())
                                                                .getNorm());

                                return branch.getScoringLocations().entrySet().stream()
                                        .filter(
                                                (location) ->
                                                        xyDistanceFromCoral
                                                                        .apply(location.getValue())
                                                                        .lt(Inches.of(2.0))
                                                                && location.getValue()
                                                                        .getMeasureZ()
                                                                        .minus(
                                                                                bottomOfCoral
                                                                                        .getMeasureZ())
                                                                        .isNear(
                                                                                Inches.of(0.0),
                                                                                Inches.of(6.0)))
                                        .findFirst();
                            });
        }

        ForbarComponentOffsets updateForbarComponentPositions(double percentOutVsIn) {
            Angle changeInShortPivotPitch =
                    config.shortPivotPitchRange.times(MathUtil.clamp(percentOutVsIn, 0.0, 1.0));

            var shortPivotOffset =
                    config.robotToShortPivot.transformBy(
                            GeomUtil.toTransform3d(0.0, changeInShortPivotPitch, 0.0));

            var endOfShortPivot =
                    shortPivotOffset
                            .transformBy(
                                    GeomUtil.toTransform3d(0.0, config.shortPivotPitchOffset, 0.0))
                            .transformBy(GeomUtil.toTransform3d(config.shortPivotLength, 0.0, 0.0));

            var endOfLongPivot =
                    GeomUtil.calcIntercetionOfCirclesInXZPlane(
                                    config.robotToLongPivot.getTranslation(),
                                    config.longPivotLength,
                                    endOfShortPivot.getTranslation(),
                                    config.backPlatePivotSeparation)
                            .get(1);

            var toEndOfLongPivot = endOfLongPivot.minus(config.robotToLongPivot.getTranslation());

            var longPivotOffset =
                    new Pose3d(
                            config.robotToLongPivot.getTranslation(),
                            new Rotation3d(
                                    0.0,
                                    Math.atan2(toEndOfLongPivot.getX(), toEndOfLongPivot.getZ()),
                                    0.0));

            var toEndOfLongPivotFromEndofShortPivot =
                    endOfLongPivot.minus(endOfShortPivot.getTranslation());

            var carriageOffset =
                    new Pose3d(
                            endOfShortPivot.getTranslation(),
                            new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(40)
                                            + Math.atan2(
                                                    toEndOfLongPivotFromEndofShortPivot.getX(),
                                                    toEndOfLongPivotFromEndofShortPivot.getZ()),
                                    0.0));

            var bottomOfCoral =
                    carriageOffset
                            .transformBy(GeomUtil.toTransform3d(0.0, Degrees.of(-90), 0.0))
                            .transformBy(
                                    GeomUtil.toTransform3d(
                                            config.carriageToBottomOfCoral.rotateBy(
                                                    new Rotation3d(
                                                            Degrees.zero(),
                                                            Degrees.of(90),
                                                            Degrees.zero()))));

            return new ForbarComponentOffsets(
                    addElevatorOffset(shortPivotOffset),
                    addElevatorOffset(longPivotOffset),
                    addElevatorOffset(carriageOffset),
                    addElevatorOffset(endOfShortPivot),
                    addElevatorOffset(new Pose3d(endOfLongPivot, Rotation3d.kZero)),
                    addElevatorOffset(bottomOfCoral));
        }

        private static Pose3d addElevatorOffset(Pose3d offset) {
            return Robot.getElevator()
                    .getState()
                    .getStage2Displacement()
                    .transformBy(GeomUtil.toTransform3d(offset));
        }
    }

    @Getter
    @RequiredArgsConstructor
    public static class ForbarComponentOffsets {
        final Pose3d shortPivot, longPivot, carriage;
        final Pose3d endOfShortPivot, endOfLongPivot, bottomOfCoral;
    }

    @Override
    public void setupBindings() {
        ForbarStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        ForbarStates.setupDefaultCommand();
    }
}
