package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JawsSubsystem extends SubsystemBase {

    private final Wheel wheel = new Wheel();
    private final WindowMotor windowMotor = new WindowMotor();

    public JawsSubsystem() {
        super();

        SmartDashboard.putData("Jaws/Jaws", this);

        SmartDashboard.putData("Jaws/Commands/Grab", grab());
        SmartDashboard.putData("Jaws/Commands/Release", release());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Jaws/Stats/Wheel Power Out (Watts)", wheel.getMotorOutputPower().in(Watts));
        SmartDashboard.putNumber("Jaws/Stats/Window Motor Power Out (Watts)",
                windowMotor.getMotorOutputPower().in(Watts));
    }

    Command setPosition(Position position) {
        return Commands.runEnd(() -> {
            switch (position) {
                case Closed:
                    windowMotor.close();
                    break;
                case Opened:
                    windowMotor.open();
                    break;
            }
        }, windowMotor::zero).until(windowMotor::isPowerSpiking);
    }

    Command wheelGrab() {
        return new FunctionalCommand(() -> {
        }, wheel::in, (interrupted) -> {
            if (interrupted)
                wheel.zero();
            else
                wheel.hold();
        }, wheel::isPowerSpiking);
    }

    public Command grab() {
        var command = Commands.deadline(
                wheelGrab(),
                setPosition(Position.Closed));

        command.addRequirements(this);

        return command;
    }

    Command wheelRelease() {
        return Commands.runEnd(wheel::out, wheel::zero).withTimeout(Seconds.of(0.25));
    }

    public Command release() {
        var command = Commands.deadline(
                wheelRelease(),
                setPosition(Position.Closed))
                .andThen(setPosition(Position.Opened));

        command.addRequirements(this);

        return command;
    }

    enum Position {
        Opened, Closed
    }

    class PowerSpikeMotor extends TalonSRX {

        final Power POWER_SPIKE_THRESHOLD;

        PowerSpikeMotor(int id, Power spikeThreshold) {
            super(id);

            POWER_SPIKE_THRESHOLD = spikeThreshold;
        }

        public Power getMotorOutputPower() {
            return Amps.of(getStatorCurrent()).times(Volts.of(Math.abs(getMotorOutputVoltage())));
        }

        public boolean isPowerSpiking() {
            return getMotorOutputPower().gt(POWER_SPIKE_THRESHOLD);
        }

        void setVoltageOutput(Voltage volts) {
            set(ControlMode.PercentOutput, volts.in(Volts) / getBusVoltage());
        }
    }

    class Wheel {
        private final PowerSpikeMotor motor = new PowerSpikeMotor(16, Watts.of(5.0));

        void hold() {
            motor.setVoltageOutput(Volts.of(3.0));
        }

        void in() {
            motor.setVoltageOutput(Volts.of(5.0));
        }

        void out() {
            motor.setVoltageOutput(Volts.of(-5.0));
        }

        void zero() {
            motor.setVoltageOutput(Volts.zero());
        }

        boolean isPowerSpiking() {
            return motor.isPowerSpiking();
        }

        Power getMotorOutputPower() {
            return motor.getMotorOutputPower();
        }
    }

    class WindowMotor {
        private final PowerSpikeMotor motor = new PowerSpikeMotor(18, Watts.of(5.0));

        void open() {
            motor.setVoltageOutput(Volts.of(-5.0));
        }

        void close() {
            motor.setVoltageOutput(Volts.of(5.0));
        }

        void zero() {
            motor.setVoltageOutput(Volts.zero());
        }

        boolean isPowerSpiking() {
            return motor.isPowerSpiking();
        }

        Power getMotorOutputPower() {
            return motor.getMotorOutputPower();
        }
    }
}
