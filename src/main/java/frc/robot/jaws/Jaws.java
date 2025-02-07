package frc.robot.jaws;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.mechanism.TalonSRXMechanism;
import lombok.Getter;

public class Jaws extends TalonSRXMechanism {
    public static class JawsConfig extends TalonSRXMechanism.Config {
        @Getter private Power spikeThreshold = Watts.of(5.0);
        @Getter private Voltage openAndCloseVoltage = Volts.of(5.0); // close is positive

        public JawsConfig() {
            super("Jaws", 18);
        }
    }

    private JawsConfig config;

    public Jaws(JawsConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Jaws/Power (Watts)", getPower().in(Watts));
    }

    private boolean isPowerSpikeExceeded() {
        return getPower().gt(config.getSpikeThreshold());
    }

    protected Command zero() {
        return run(() -> setVoltageOut(Volts.zero()));
    }

    protected Command open() {
        return run(() -> setVoltageOut(config.getOpenAndCloseVoltage().unaryMinus()))
                .until(this::isPowerSpikeExceeded);
    }

    protected Command close() {
        return run(() -> setVoltageOut(config.getOpenAndCloseVoltage()))
                .until(this::isPowerSpikeExceeded);
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
