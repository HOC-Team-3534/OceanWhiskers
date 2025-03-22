package frc.robot.subsystems.jaws;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.GeomUtil;
import frc.robot.Robot;
import lombok.Getter;
import lombok.Setter;

public class Jaws extends TalonSRXMechanism {
    @Getter
    public static class JawsConfig extends TalonSRXMechanism.Config {
        @Getter private Power spikeThreshold = Watts.of(Double.POSITIVE_INFINITY);
        @Getter private Voltage inAndOutVoltage = Volts.of(8.3); // out is positive

        Pose3d fieldToRobotCAD = new Pose3d(0.0, 0.0, 0.0414, new Rotation3d());

        Pose3d robotToAlgaeArm =
                fieldToRobotCAD.transformBy(
                        GeomUtil.toTransform3d(new Translation3d(0.28, 0.0, 0.38)));

        Pose3d robotToAlgaeArmOut =
                robotToAlgaeArm.transformBy(GeomUtil.toTransform3d(0.0, Degrees.of(-90 - 15), 0.0));

        Time timeFromInToOut = Seconds.of(1.5);

        public JawsConfig() {
            super("Jaws", 17);
        }
    }

    private JawsConfig config;

    @Getter private State state = new State();

    public Jaws(JawsConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            motor.setInverted(false);
        }
    }

    @Override
    public void periodic() {

        Logging.log("Jaws", this);
    }

    private boolean isPowerSpikeExceeded() {
        return getPower().gt(config.getSpikeThreshold())
                || state.getPositionTimer().hasElapsed(config.timeFromInToOut.in(Seconds));
    }

    protected Command zero() {
        return run(
                () ->
                        setVoltageOut(
                                state.isOut() ? config.inAndOutVoltage.times(0.25) : Volts.zero()));
    }

    protected Command in() {
        return startRun(
                        () -> state.setPosition(Position.GoingIn),
                        () -> setVoltageOut(config.getInAndOutVoltage().unaryMinus()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.In)));
    }

    protected Command out() {
        return startRun(
                        () -> state.setPosition(Position.GoingOut),
                        () -> setVoltageOut(config.getInAndOutVoltage()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.Out)));
    }

    public enum Position {
        In,
        Out,
        GoingIn,
        GoingOut
    }

    @Getter
    @Setter
    public class State {
        Position position = Position.In;
        Timer positionTimer = new Timer();
        CachedValue<Pose3d> componentOffset = new CachedValue<>(() -> updateComponentOffset());

        public void setPosition(Position position) {
            if (!this.position.equals(position)) {
                positionTimer.restart();
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
                    yield 1 - Math.min(positionTimer.get() / config.timeFromInToOut.in(Seconds), 1);
                case GoingOut:
                    yield Math.min(positionTimer.get() / config.timeFromInToOut.in(Seconds), 1);
                case In:
                    yield 0.0;
                case Out:
                    yield 1.0;
            };
        }

        public Pose3d getComponentOffset() {
            return componentOffset.get();
        }

        private Pose3d updateComponentOffset() {
            return Robot.getElevator()
                    .getState()
                    .getStage2Displacement()
                    .transformBy(
                            GeomUtil.toTransform3d(
                                    config.robotToAlgaeArm.interpolate(
                                            config.robotToAlgaeArmOut, getPercentOutVsIn())));
        }
    }

    @Override
    public void setupBindings() {
        JawsStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        JawsStates.setupDefaultCommand();
    }
}
