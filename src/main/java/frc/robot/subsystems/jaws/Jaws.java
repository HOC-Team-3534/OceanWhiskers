package frc.robot.subsystems.jaws;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.util.GeomUtil;
import lombok.Getter;
import lombok.Setter;

public class Jaws extends TalonSRXMechanism {
    public static class JawsConfig extends TalonSRXMechanism.Config {
        @Getter private Power spikeThreshold = Watts.of(60.0);
        @Getter private Voltage inAndOutVoltage = Volts.of(8.3); // out is positive

        Pose3d fieldToRobotCAD = new Pose3d(0.0, 0.0, 0.0414, new Rotation3d());

        Pose3d robotToAlgaeArm =
                fieldToRobotCAD.transformBy(
                        GeomUtil.toTransform3d(new Translation3d(0.28, 0.0, 0.38)));

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
        if (state.isOut()) {
            setVoltageOut(config.inAndOutVoltage.times(0.25));
        }

        Logging.log("Jaws", this);
    }

    private boolean isPowerSpikeExceeded() {
        return getPower().gt(config.getSpikeThreshold());
    }

    protected Command zero() {
        return run(() -> setVoltageOut(Volts.zero()));
    }

    protected Command in() {
        return startRun(
                        () -> state.setPosition(Position.InBetween),
                        () -> setVoltageOut(config.getInAndOutVoltage().unaryMinus()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.In)));
    }

    protected Command out() {
        return startRun(
                        () -> state.setPosition(Position.InBetween),
                        () -> setVoltageOut(config.getInAndOutVoltage()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.Out)));
    }

    public enum Position {
        In,
        Out,
        InBetween
    }

    @Getter
    @Setter
    public class State {
        Position position = Position.In;

        public boolean isOut() {
            return position.equals(Position.Out);
        }

        public boolean isIn() {
            return position.equals(Position.In);
        }

        public Pose3d getComponentOffset() {
            return config.robotToAlgaeArm;
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
