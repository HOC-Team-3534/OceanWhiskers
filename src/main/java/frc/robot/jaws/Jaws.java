package frc.robot.jaws;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.mechanism.TalonSRXMechanism;
import lombok.Getter;
import lombok.Setter;

public class Jaws extends TalonSRXMechanism {
    public static class JawsConfig extends TalonSRXMechanism.Config {
        // TODO: tune power spike and output voltage levels
        @Getter private Power spikeThreshold = Watts.of(60.0);
        @Getter private Voltage openAndCloseVoltage = Volts.of(8.3); // close is positive

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
        SmartDashboard.putNumber("Jaws/Power (Watts)", getPower().in(Watts));
        SmartDashboard.putNumber("Jaws/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Jaws/Current", getCurrent().in(Amps));
    }

    private boolean isPowerSpikeExceeded() {
        return getPower().gt(config.getSpikeThreshold());
    }

    protected Command zero() {
        return run(() -> setVoltageOut(Volts.zero()));
    }

    // TODO: validate open and close happen when theya re supposed to and not when elevator is not
    // raised

    protected Command open() {
        return startRun(
                        () -> state.setPosition(Position.InBetween),
                        () -> setVoltageOut(config.getOpenAndCloseVoltage().unaryMinus()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.Opened)));
    }

    protected Command close() {
        return startRun(
                        () -> state.setPosition(Position.InBetween),
                        () -> setVoltageOut(config.getOpenAndCloseVoltage()))
                .until(this::isPowerSpikeExceeded)
                .andThen(runOnce(() -> state.setPosition(Position.Closed)));
    }

    public enum Position {
        Opened,
        Closed,
        InBetween
    }

    @Getter
    @Setter
    public class State {
        Position position = Position.Opened;

        public boolean isClosed() {
            return position.equals(Position.Closed);
        }

        public boolean isOpened() {
            return position.equals(Position.Opened);
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
