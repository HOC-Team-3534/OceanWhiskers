package frc.robot.subsystems.forbar;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.util.GeomUtil;
import frc.robot.Robot;
import lombok.Getter;
import lombok.Setter;

public class Forbar extends TalonSRXMechanism {
    public static class ForbarConfig extends TalonSRXMechanism.Config {
        @Getter private Voltage outVoltage = Volts.of(12.0); // out is positive
        @Getter private Voltage holdOutVoltage = Volts.of(4.0); // out is positive
        @Getter private Voltage inVoltage = Volts.of(-7.0); // in is negative
        @Getter private Power spikeThreshold = outVoltage.times(Amps.of(10.0));

        @Getter private Pose3d bottomBarIn = new Pose3d();
        @Getter private Pose3d bottomBarOut = new Pose3d();
        @Getter private Pose3d topBarIn = new Pose3d();
        @Getter private Pose3d topBarOut = new Pose3d();
        @Getter private Pose3d carriageIn = new Pose3d();
        @Getter private Pose3d carriageOut = new Pose3d();

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
        Position position = Position.In;
        Pose3d currentOffset = new Pose3d();
        Timer currentPositionTimer = new Timer();

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

        private Pose3d interpolatePose(Pose3d in, Pose3d out) {
            return in.interpolate(out, getPercentOutVsIn());
        }

        private Pose3d baseElevatorOffset() {
            return new Pose3d(
                    0.0, 0.0, Robot.getElevator().getHeight().in(Meters), Rotation3d.kZero);
        }

        public Pose3d getBottomBar() {
            return baseElevatorOffset()
                    .transformBy(
                            GeomUtil.toTransform3d(
                                    interpolatePose(config.bottomBarIn, config.bottomBarOut)));
        }

        public Pose3d getTopBar() {
            return baseElevatorOffset()
                    .transformBy(
                            GeomUtil.toTransform3d(
                                    interpolatePose(config.topBarIn, config.topBarOut)));
        }

        public Pose3d getCarriage() {
            return baseElevatorOffset()
                    .transformBy(
                            GeomUtil.toTransform3d(
                                    interpolatePose(config.carriageIn, config.carriageOut)));
        }
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
