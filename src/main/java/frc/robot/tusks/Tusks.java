package frc.robot.tusks;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.hocLib.mechanism.TalonSRXMechanism;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Tusks extends TalonSRXMechanism {

    public static class TusksConfig extends TalonSRXMechanism.Config {
        @Getter private boolean motionMagicEnabled;

        @Getter private Angle up = Degrees.of(90);
        @Getter private Angle pickup = Degrees.of(30);
        @Getter private Angle preDeploy = Degrees.of(50);
        @Getter private Angle deploy = Degrees.of(-20);

        @Getter @Setter ArmFeedforward ff_noCoral = new ArmFeedforward(0, 0, 0);
        @Getter @Setter ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0);

        public TusksConfig() {
            super("Tusks", 17, 4096, 1.0 / 100.0);

            var talonConfig = getTalonConfig();

            var slotConfig = new SlotConfiguration();

            slotConfig.kP = 0.0;
            slotConfig.kI = 0.0;
            slotConfig.kD = 0.0;

            talonConfig.slot0 = slotConfig;

            setMMConfigs(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(1.0), 2);
        }

        public TusksConfig enableMotionMagic() {
            this.motionMagicEnabled = true;
            return this;
        }
    }

    private SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(null, Volts.of(2), null, (state) -> {}),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    final State state = new State();

    private TusksConfig config;

    public Tusks(TusksConfig config) {
        super(config);
        this.config = config;

        motor.setSensorPhase(true);

        motor.setSelectedSensorPosition(positionToPositionInSensorTicks(Degrees.of(90)));
    }

    @Override
    public void periodic() {
        if (!state.isHoldingCoral() // has no coral
                && getVelocity().lt(DegreesPerSecond.zero()) // tusks are moving down
                && getError().gt(Degrees.of(5))) { // tusks are far below target angle
            state.setHoldingCoral(true); //
        }

        if (getPosition().lt(Degrees.of(-15))) {
            state.setHoldingCoral(false);
        }

        SmartDashboard.putNumber("Tusks/Angle (Deg.)", getPosition().in(Degrees));
        SmartDashboard.putNumber("Tusks/Output Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Tusks/Velocity (RPS)", getVelocity().in(RotationsPerSecond));
    }

    public enum Side {
        Left,
        Right
    }

    private void logMotor(SysIdRoutineLog log) {
        log.motor("tusks")
                .voltage(getVoltage())
                .angularPosition(getPosition())
                .angularVelocity(getVelocity());
    }

    public Command up() {
        return run(() -> setAngle(Degrees.of(90)));
    }

    public Command pickup() {
        return run(() -> setAngle(Degrees.of(20)));
    }

    public Command preDeploy() {
        return run(() -> setAngle(Degrees.of(50)));
    }

    public Command deploy() {
        return run(() -> setAngle(Degrees.of(-40)));
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> setVoltageOut(voltsSupplier.get()));
    }

    Angle getError() {
        return positionInSensorTicksToPosition(
                motor.getClosedLoopTarget() - motor.getSelectedSensorPosition());
    }

    ArmFeedforward getCurrentFF() {
        return (state.isHoldingCoral()) ? config.getFf_withCoral() : config.getFf_noCoral();
    }

    double getArbitraryFF() {
        return getCurrentFF().calculate(getPosition().in(Radians), motor.getClosedLoopError())
                / motor.getBusVoltage();
    }

    void setAngle(Angle angle) {
        if (isAttached()) {
            var ticks = positionToPositionInSensorTicks(angle);
            if (config.isMotionMagicEnabled())
                motor.set(
                        ControlMode.MotionMagic,
                        ticks,
                        DemandType.ArbitraryFeedForward,
                        getArbitraryFF());
            else zero();
        }
    }

    protected void zero() {
        setVoltageOut(Volts.zero());
    }

    @Override
    protected void setVoltageOut(Voltage voltage) {
        if (getPosition().gt(Degrees.of(85))) voltage = Volts.of(Math.min(voltage.in(Volts), 0.0));
        if (getPosition().lt(Degrees.of(-50))) voltage = Volts.of(Math.max(voltage.in(Volts), 0.0));
        super.setVoltageOut(voltage);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public State getState() {
        return state;
    }

    public class State {
        @Getter @Setter private boolean holdingCoral;

        public boolean isReadyToDeploy() {
            return getPosition().minus(Degrees.of(3.0)).lt(config.getPreDeploy());
        }
    }

    @Override
    public void setupBindings() {
        TusksStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        TusksStates.setupDefaultCommand();
    }
}
