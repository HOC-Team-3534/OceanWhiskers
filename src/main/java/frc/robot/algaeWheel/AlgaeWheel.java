package frc.robot.algaeWheel;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.mechanism.TalonSRXMechanism;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class AlgaeWheel extends TalonSRXMechanism {
    @Getter
    @Setter
    @Accessors(chain = true)
    public static class AlgaeWheelConfig extends TalonSRXMechanism.Config {
        Power spikeThreshold = Watts.of(Double.POSITIVE_INFINITY);
        // TODO: tune power on wheel for removing algae from reef
        Voltage grabVoltage = Volts.of(12.0);
        Voltage releaseVoltage = Volts.of(-7.0);
        Voltage holdVoltage = Volts.of(0.3);

        public AlgaeWheelConfig() {
            super("Algae Wheel", 16);
        }
    }

    private AlgaeWheelConfig config;

    @Getter private State state = new State();

    public AlgaeWheel(AlgaeWheelConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void periodic() {
        if (isPowerSpikeExceeded() && getVoltage().gt(Volts.zero())) {
            state.setHoldingBall(true);
        }

        if (getVoltage().lt(Volts.zero())) {
            state.setHoldingBall(false);
        }

        SmartDashboard.putNumber("Algae Wheel/Power (Watts)", getPower().in(Watts));
    }

    private boolean isPowerSpikeExceeded() {
        return getPower().gt(config.getSpikeThreshold());
    }

    protected Command zero() {
        return run(() -> setVoltageOut(Volts.zero()));
    }

    protected Command grab() {
        return run(() -> setVoltageOut(config.getGrabVoltage()));
    }

    protected Command release() {
        return run(() -> setVoltageOut(config.getReleaseVoltage()));
    }

    protected Command hold() {
        return run(() -> setVoltageOut(config.getHoldVoltage()));
    }

    public static class State {
        @Getter @Setter private boolean holdingBall;
    }

    @Override
    public void setupBindings() {
        AlgaeWheelStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        AlgaeWheelStates.setupDefaultCommand();
    }
}
