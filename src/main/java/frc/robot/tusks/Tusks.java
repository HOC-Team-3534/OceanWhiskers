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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.motionmagic.MotionMagicV5;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Tusks extends TalonSRXMechanism {

    public static class TusksConfig extends TalonSRXMechanism.Config {
        @Getter private boolean motionMagicEnabled;

        @Getter @Setter ArmFeedforward ff_noCoral = new ArmFeedforward(0, 0, 0);
        @Getter @Setter ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0);

        public TusksConfig() {
            super("Tusks", 17, 4096, 1.0 / 100.0);

            var talonConfig = getTalonConfig();

            var slotConfig = new SlotConfiguration();

            slotConfig.kP = 0.0;
            slotConfig.kI = 0.0;
            slotConfig.kD = 0.0;

            slotConfig.kF = MotionMagicV5.convV6toV5(0.0, Volts.of(12), 4096);

            talonConfig.slot0 = slotConfig;

            var slot1Config = new SlotConfiguration();

            slot1Config.kP = 0.0;
            slot1Config.kI = 0.0;
            slot1Config.kD = 0.0;

            slot1Config.kF = MotionMagicV5.convV6toV5(0.0, Volts.of(12), 4096);

            talonConfig.slot1 = slot1Config;

            setMMConfigs(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(1.0), 2);
        }

        public TusksConfig enableMotionMagic() {
            this.motionMagicEnabled = true;
            return this;
        }
    }

    final State state = new State();

    private TusksConfig config;

    public Tusks(TusksConfig config) {
        super(config);
        this.config = config;

        motor.setSelectedSensorPosition(positionToPositionInSensorTicks(Degrees.of(90)));
    }

    @Override
    public void periodic() {
        if (!state.hasCoral() // has no coral
                && getVelocity()
                        .lt(
                                DegreesPerSecond
                                        .zero()) // tusks are moving down despite the motor moving
                // up
                && getError().gt(Degrees.of(5))) { // tusks are far below target angle
            state.setHasCoral(true); //
        }

        if (getPosition().lt(Degrees.of(-15))) {
            state.setHasCoral(false);
        }

        SmartDashboard.putNumber("Tusks/Stats/Angle (Deg.)", getPosition().in(Degrees));
        SmartDashboard.putNumber("Tusks/Stats/Output Voltage", getVoltage().in(Volts));
        SmartDashboard.putBoolean("Tusks/Stats/Deploying", state.isDeploying());
        SmartDashboard.putNumber(
                "Tusks/Stats/Velocity (RPS)", getVelocity().in(RotationsPerSecond));
    }

    public enum Side {
        Left,
        Right
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
        return runEnd(
                        () -> {
                            if (deploy.get()) state.setDeploying(true);

                            if (state.isDeploying()) setAngle(Degrees.of(-30));
                            else setAngle(Degrees.of(90));
                        },
                        () -> state.notDeploying())
                .until(() -> !state.hasCoral());
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> setVoltageOut(voltsSupplier.get()));
    }

    Angle getError() {
        return positionInSensorTicksToPosition(
                motor.getClosedLoopTarget() - motor.getSelectedSensorPosition());
    }

    ArmFeedforward getCurrentFF() {
        return (state.hasCoral()) ? config.getFf_withCoral() : config.getFf_noCoral();
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
        if (getPosition().gt(Degrees.of(90))) voltage = Volts.of(Math.min(voltage.in(Volts), 0.0));
        if (getPosition().lt(Degrees.of(-50))) voltage = Volts.of(Math.max(voltage.in(Volts), 0.0));
        super.setVoltageOut(voltage);
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

    @Override
    public void setupBindings() {
        TusksStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        TusksStates.setupDefaultCommand();
    }
}
