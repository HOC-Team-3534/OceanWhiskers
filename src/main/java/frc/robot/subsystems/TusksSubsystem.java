package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TusksSubsystem extends SubsystemBase {
    final TalonSRX tusks = new TalonSRX(17);

    final ArmFeedforward ff_noCoral = new ArmFeedforward(0, 0, 0);
    final ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0.0);

    final double GEAR_RATIO = 100.0 / 1.0; // 100 rotations of motor is 1 rotation of tusk
    final double TICKS_PER_REV = 4096.0;

    final State state = new State();
    @SuppressWarnings("unused")
    private final Telemetry telemetry = new Telemetry(this);

    private final boolean DISABLE_MOTION_MAGIC = true;

    public TusksSubsystem() {
        super();

        setDefaultCommand(off());

        var config = new TalonSRXConfiguration();

        config.motionCruiseVelocity = 0.0;
        config.motionAcceleration = 0.0;
        config.motionCurveStrength = 1;

        var slotConfig = new SlotConfiguration();

        slotConfig.kP = 0.0;
        slotConfig.kI = 0.0;
        slotConfig.kD = 0.0;

        config.slot0 = slotConfig;

        tusks.configAllSettings(config);

        tusks.setInverted(false);
        tusks.setSensorPhase(false);

        tusks.setSelectedSensorPosition(fromAngle(Degrees.of(90)));
    }

    @Override
    public void periodic() {
        if (!state.hasCoral() //has no coral
                && getVelocity().lt(DegreesPerSecond.zero()) //tusks are moving down despite the motor moving up
                && getError().gt(Degrees.of(5))) { // tusks are far below target angle
            state.setHasCoral(true);// 
        }

        if (getAngle().lt(Degrees.of(-15))) {
            state.setHasCoral(false);
        }
    }

    double fromAngle(Angle angle) {
        return angle.times(GEAR_RATIO).in(Rotations) * TICKS_PER_REV;

    }

    Angle toAngle(double ticks) {
        return Rotations.of(ticks / TICKS_PER_REV).div(GEAR_RATIO);
    }

    public Angle getAngle() {
        return toAngle(tusks.getSelectedSensorPosition());
    }

    AngularVelocity getVelocity() {
        return toAngle(tusks.getSelectedSensorVelocity()).div(Milliseconds.of(100));
    }

    Angle getError() {
        return toAngle(tusks.getClosedLoopTarget() - tusks.getSelectedSensorPosition());
    }

    ArmFeedforward getCurrantFF() {
        return (state.hasCoral()) ? ff_withCoral : ff_noCoral;
    }

    void setAngle(Angle angle) {
        var ticks = fromAngle(angle);
        if (!DISABLE_MOTION_MAGIC)
            tusks.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward,
                    getCurrantFF().calculate(getAngle().in(Radians), getVelocity().in(RadiansPerSecond)));
        else
            setVoltageZero();
    }

    public Command off() {
        return run(() -> setVoltageZero());
    }

    void setVoltageZero() {
        tusks.set(ControlMode.PercentOutput, 0);
    }

    public Command up() {
        return run(() -> setAngle(Degrees.of(90)));
    }

    public Command pickup() {
        return run(() -> setAngle(Degrees.of(20)));
    }

    public Command deploy() {
        return deploy(() -> true);
    }

    public Command deploy(Supplier<Boolean> deploy) {
        return runEnd(() -> {
            if (deploy.get())
                state.setDeploying(true);

            if (state.isDeploying())
                setAngle(Degrees.of(-30));
            else
                setAngle(Degrees.of(90));
        }, () -> state.notDeploying())
                .until(() -> !state.hasCoral());
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> tusks.set(ControlMode.PercentOutput, voltsSupplier.get().in(Volts) / tusks.getBusVoltage()));
    }

    public State getState() {
        return state;
    }

    public class State {
        boolean hasCoral;
        Timer angleDownTimer = new Timer();
        boolean deploying;

        void setHasCoral(boolean hasCoral) {
            this.hasCoral = hasCoral;
        }

        public boolean hasCoral() {
            return hasCoral;
        }

        boolean isDeploying() {
            return deploying;
        }

        void setDeploying(boolean deploying) {
            this.deploying = deploying;
        }

        void notDeploying() {
            deploying = false;
        }

    }

    public class Telemetry {
        final ShuffleboardLayout tusksCommands = Shuffleboard.getTab("Commands").getLayout("Tusks",
                BuiltInLayouts.kList);

        final ShuffleboardLayout tusksStats = Shuffleboard.getTab("Commands").getLayout("Tusks Stats",
                BuiltInLayouts.kList);

        final GenericEntry voltageOutEntry = tusksCommands.add("Raw Voltage Out", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider).getEntry();

        Telemetry(TusksSubsystem tusks) {
            tusksCommands.add("Pickup", pickup());
            tusksCommands.add("Deploy", deploy());

            tusksCommands.add("Apply Voltage Out", voltageOut(() -> Volts.of(voltageOutEntry.getDouble(0.0))));

            tusksStats.addDouble("Angle (Deg)", () -> getAngle().in(Degrees));
            tusksStats.addDouble("Raw Ticks", tusks.tusks::getSelectedSensorPosition);
            tusksStats.addDouble("Output Voltage", tusks.tusks::getMotorOutputVoltage);
            tusksStats.addBoolean("Deploying", tusks.state::isDeploying);
        }

    }
}
