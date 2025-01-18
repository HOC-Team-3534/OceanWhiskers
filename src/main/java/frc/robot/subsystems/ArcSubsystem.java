package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArcSubsystem extends SubsystemBase {
    private final TalonFX arc = new TalonFX(16);

    private final Power INTAKE_POWER_LIMIT = Watts.of(5.0);//TODO: tune power limit

    private final State state = new State();

    public ArcSubsystem() {
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
        var powerDraw = arc.getStatorCurrent().getValue().times(arc.getMotorVoltage().getValue());
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
        arc.setControl(new VoltageOut(volts));
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
