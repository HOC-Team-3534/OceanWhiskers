package frc.robot.subsystems.forbar;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.GeomUtil;
import frc.robot.Robot;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Forbar extends TalonSRXMechanism {
    public static class ForbarConfig extends TalonSRXMechanism.Config {
        @Getter private Voltage outVoltage = Volts.of(12.0); // out is positive
        @Getter private Voltage holdOutVoltage = Volts.of(4.0); // out is positive
        @Getter private Voltage inVoltage = Volts.of(-7.0); // in is negative
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

        @Getter private Time timeFromInToOut = Seconds.of(0.5);

        public ForbarConfig() {
            super("Forbar", 16);
        }
    }

    private ForbarConfig config;

    @Getter private State state = new State();

    public Forbar(ForbarConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            motor.setInverted(false);
        }
    }

    @Override
    public void periodic() {
        Logging.log("Forbar", this);
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
