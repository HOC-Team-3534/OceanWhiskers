package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JawsSubsystem extends SubsystemBase {
    private final TalonSRX jaws = new TalonSRX(16);

    private final Power INTAKE_POWER_LIMIT = Watts.of(5.0);//TODO: tune power limit

    private final State state = new State();

    @SuppressWarnings("unused")
    private final Telemetry telemetry = new Telemetry(this);

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
        if (getMotorOutputPower().gt(INTAKE_POWER_LIMIT)) {
            state.grabbedBall();
        }
    }

    public Command grab() {
        return runEnd(() -> setVoltageOut(Volts.of(7)), this::zero);
    }

    public Command realese() {
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

    Power getMotorOutputPower() {
        return Amps.of(jaws.getStatorCurrent()).times(Volts.of(jaws.getMotorOutputVoltage()));
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

    public class Telemetry {
        final ShuffleboardLayout jawsCommands = Shuffleboard.getTab("Commands").getLayout("Jaws Stats",
                BuiltInLayouts.kList);
        final ShuffleboardLayout jawsStats = Shuffleboard.getTab("Commands").getLayout("Jaws Stats",
                BuiltInLayouts.kList);

        Telemetry(JawsSubsystem jaws) {
            jawsCommands.add("Grab", grab());
            jawsCommands.add("Realese", realese());

            jawsStats.addDouble("Motor Output Power", () -> jaws.getMotorOutputPower().in(Watts))
                    .withWidget(BuiltInWidgets.kGraph);
            jawsStats.addBoolean("Holding Ball", () -> jaws.state.isHoldingBall());
            jawsStats.addDouble("Motor Output Voltage", () -> jaws.jaws.getMotorOutputVoltage());
        }
    }
}
