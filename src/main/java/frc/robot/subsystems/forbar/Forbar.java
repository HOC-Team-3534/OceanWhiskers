package frc.robot.subsystems.forbar;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import lombok.Getter;
import lombok.Setter;

public class Forbar extends TalonSRXMechanism {
    public static class ForbarConfig extends TalonSRXMechanism.Config {
        @Getter private Power spikeThreshold = Watts.of(30.0);
        @Getter private Voltage inAndOutVoltage = Volts.of(1.0); // out is positive

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
        return getPower().gt(config.getSpikeThreshold());
    }

    protected Command zero() {
        return run(() -> setVoltageOut(Volts.zero()));
    }

    // TODO: validate open and close happen when theya re supposed to and not when elevator is not
    // raised

    protected Command in() {
        return Commands.waitSeconds(0.0)
                .andThen(
                        startRun(
                                        () -> state.setPosition(Position.InBetween),
                                        () ->
                                                setVoltageOut(
                                                        config.getInAndOutVoltage().unaryMinus()))
                                .until(this::isPowerSpikeExceeded),
                        runOnce(() -> state.setPosition(Position.In)));
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
