package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.hocLib.mechanism.TalonFXMechanism;
import lombok.Getter;

public class Elevator extends TalonFXMechanism {

    public static class ElevatorConfig extends Config {
        @Getter private Distance L1 = Inches.of(10);
        @Getter private Distance L2 = Inches.of(20);
        @Getter private Distance L3 = Inches.of(30);
        @Getter private Distance L4 = Inches.of(40);
        @Getter private Distance PickUp = Inches.of(15);

        @Getter private Distance Deploy = Inches.of(5.0);

        @Getter private boolean motionMagicEnabled = false;

        public ElevatorConfig() {
            super("Elevator Leader", 14);
            setFollowerConfigs(new FollowerConfig("Elevator Follower", 15, true));

            var slot0Configs = new Slot0Configs();

            slot0Configs.kP = 0.25;
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            slot0Configs.kG = 0;
            slot0Configs.kS = 0;
            slot0Configs.kV = 0;
            slot0Configs.kA = 0;

            setSlot0Configs(slot0Configs);

            setMMConfigs(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(15.0))
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15.0))
                            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)));

            setMotorOutputConfigs(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

            configMaxLinearPosition(Inches.of(54.625));
            configMaxPosition(Rotations.of(23.713));
        }
    }

    private final State state = new State();

    private final VoltageOut voltageOut = new VoltageOut(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private ElevatorConfig config;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Stats/Angle (Deg)", getPosition().in(Degrees));
        SmartDashboard.putNumber("Elevator/Stats/Height (In.)", getHeight().in(Inches));
        SmartDashboard.putNumber(
                "Elevator/Stats/Target Height (In.)", getTargetHeight().in(Inches));
        SmartDashboard.putBoolean("Elevator/Stats/Near Target Height", state.isNearTargetHeight());
        SmartDashboard.putBoolean("Elevator/Stats/Deploying", state.isDeploying());
        SmartDashboard.putNumber("Elevator/Stats/Voltage Output", getVoltage().in(Volts));
        SmartDashboard.putNumber(
                "Elevator/Stats/Velocity (RPS)", getVelocity().in(RotationsPerSecond));
    }

    @Override
    public String getName() {
        return "Elevator";
    }

    public Distance getHeight() {
        return getLinearPosition();
    }

    void slowlyLower() {
        setVoltageOutput(Volts.of(0.35));
    }

    void zero() {
        setVoltageOutput(Volts.zero());
    }

    private Distance getTargetHeight() {
        return state.getTargetLevel()
                .getHeight(config)
                .plus(state.isDeploying() ? config.getDeploy() : Inches.zero());
    }

    private Angle getTargetRawPosition() {
        return linearPositionToPosition(getTargetHeight());
    }

    void setVoltageOutput(Voltage volts) {
        if (isAttached()) {
            if (getHeight().gt(config.getMaxLinearPosition().minus(Inches.of(3.0)))) {
                volts = Volts.of(0.35);
            }
            if (volts.lt(Volts.zero())) {
                volts = Volts.of(0.35);
            }
            motor.setControl(voltageOut.withOutput(volts));
        }
    }

    void updateHeight() {
        if (isAttached()) {
            if (config.isMotionMagicEnabled()) {
                if (state.isNearTargetHeight() || state.isClimbing()) zero();
                else motor.setControl(motionMagicVoltage.withPosition(getTargetRawPosition()));
            } else {
                if (getHeight().lt(Inches.of(1.0))) {
                    zero();
                } else {
                    slowlyLower();
                }
            }
        }
    }

    public State getState() {
        return state;
    }

    public enum Level {
        Bottom,
        L1,
        L2,
        L3,
        L4,
        PickUp;

        Distance getHeight(ElevatorConfig config) {
            switch (this) {
                case Bottom:
                    return Inches.of(0);
                case L1:
                    return config.getL1();
                case L2:
                    return config.getL2();
                case L3:
                    return config.getL3();
                case L4:
                    return config.getL4();
                case PickUp:
                    return config.getPickUp();
                default:
                    return Inches.of(0);
            }
        }
    }

    public class State {
        private boolean climbing = false;
        private Level targetLevel = Level.Bottom;
        private boolean deploying;

        public boolean isClimbing() {
            return climbing;
        }

        void setClimbing(boolean climbing) {
            this.climbing = climbing;
        }

        public Level getTargetLevel() {
            return targetLevel;
        }

        void setTargetLevel(Level targetHeight) {
            this.targetLevel = targetHeight;
        }

        boolean isDeploying() {
            return deploying;
        }

        void setDeploying(boolean deploying) {
            this.deploying = deploying;
        }

        public boolean isReefTargetHeight() {
            switch (targetLevel) {
                case L1, L2, L3, L4:
                    return true;
                default:
                    return false;
            }
        }

        public boolean isNearTargetHeight() {
            return getHeight().isNear(getTargetHeight(), Inches.of(0.5));
        }
    }

    @Override
    public void setupTriggeringOfCommands() {
        ElevatorStates.setupTriggeringOfCommands();
    }

    @Override
    public void setupDefaultCommand() {
        ElevatorStates.setupDefaultCommand();
    }
}
