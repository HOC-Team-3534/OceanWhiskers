package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JawsSubsystem extends SubsystemBase {
    private final TalonSRX jaws = new TalonSRX(16);

    private final Power INTAKE_POWER_LIMIT = Watts.of(5.0);//TODO: tune power limit

    private final State state = new State();

    public JawsSubsystem() {
        super();

        setDefaultCommand(runEnd(() -> {
            if (state.isHoldingBall())
                setVoltageOut(Volts.of(3));
            else
                zero();

        }, this::zero));
    }

    @Override
    public void periodic() {
        var powerDraw = Amps.of(jaws.getStatorCurrent()).times(Volts.of(jaws.getMotorOutputVoltage()));
        if (powerDraw.gt(INTAKE_POWER_LIMIT)) {
            state.grabbedBall();
        }
    }

    public Command intake() {
        return runEnd(() -> setVoltageOut(Volts.of(7)), this::zero);
    }

    public Command extake() {
        return runEnd(() -> setVoltageOut(Volts.of(-7)), () -> {
            zero();
            state.releasedBall();
        });
    }

    private void setVoltageOut(Voltage volts) {
        jaws.set(ControlMode.PercentOutput, volts.in(Volts) / jaws.getBusVoltage());
    }

    private void zero() {
        setVoltageOut(Volts.zero());
    }

    public class State {
        private boolean holdingBall;

        public boolean isHoldingBall() {
            return holdingBall;
        }

        public void grabbedBall() {
            holdingBall = true;
        }

        public void releasedBall() {
            holdingBall = false;
        }
    }
}
