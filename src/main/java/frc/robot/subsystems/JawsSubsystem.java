package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JawsSubsystem extends SubsystemBase {

    private final State state = new State();

    private final Wheel wheel = new Wheel();
    private final WindowMotor windowMotor = new WindowMotor();

    public JawsSubsystem() {
        super();

        setDefaultCommand(run(() -> {
            if (state.isAtTargetPosition()) {
                windowMotor.zero();
            } else {
                switch (state.getTargePosition()) {
                    case Opened:
                        windowMotor.open();
                        break;

                    case Closed:
                        windowMotor.close();
                        break;

                }
            }

            if (state.isHoldingBall()) {
                wheel.hold();
            }
        }));
    }

    @Override
    public void periodic() {
        if (!state.isAtTargetPosition() && windowMotor.isPowerSpiking()) {
            state.setCurrentPosition(state.getTargePosition());
        }
    }

    Command setPosition(Position position) {
        return Commands.runOnce(() -> state.setTargetPosition(position))
                .andThen(Commands.waitUntil(state::isAtTargetPosition));
    }

    Command openJaws() {
        return setPosition(Position.Opened);
    }

    Command closeJaws() {
        return setPosition(Position.Closed);
    }

    enum Position {
        Opened, Closed
    }

    public class State {
        private boolean holdingBall;
        private Position currentPosition = Position.Opened;
        private Position targetPosition = Position.Opened;

        public boolean isHoldingBall() {
            return holdingBall;
        }

        public void grabbedBall() {
            holdingBall = true;
        }

        public void releasedBall() {
            holdingBall = false;
        }

        Position getCurrentPosition() {
            return currentPosition;
        }

        void setCurrentPosition(Position position) {
            currentPosition = position;
        }

        Position getTargePosition() {
            return targetPosition;
        }

        void setTargetPosition(Position position) {
            targetPosition = position;
        }

        boolean isAtTargetPosition() {
            return currentPosition.equals(targetPosition);
        }
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

    }
}
