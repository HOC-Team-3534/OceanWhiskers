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
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.hocLib.mechanism.TalonFXMechanism;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends TalonFXMechanism {

    public static class ElevatorConfig extends Config {
        @Getter private Distance L1 = Inches.of(10);
        @Getter private Distance L1Pre = Inches.of(5.0);
        @Getter private Distance L2 = Inches.of(20);
        @Getter private Distance L2Pre = Inches.of(15.0);
        @Getter private Distance L3 = Inches.of(30);
        @Getter private Distance L3Pre = Inches.of(25);
        @Getter private Distance L4 = Inches.of(40);
        @Getter private Distance L4Pre = Inches.of(35);
        @Getter private Distance PickUp = Inches.of(15);

        @Getter private boolean motionMagicEnabled;

        public ElevatorConfig() {
            super("Elevator Leader", 14);
            setFollowerConfigs(new FollowerConfig("Elevator Follower", 15, true));

            var slot0Configs = new Slot0Configs();

            // TODO: transfer tuning from old-main and/or retune using sysid
            slot0Configs.kP = 3.0;
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            slot0Configs.kG = 0.805;
            slot0Configs.kS = 0.0559;
            slot0Configs.kV = 0.11793;
            slot0Configs.kA =
                    0.01689; // TODO: try with a kA that meets the kG / kA is 9.8 m/s sqrd rule
            // with 0.805 kG, kA should be more like 0.0048

            setSlot0Configs(slot0Configs);

            setMMConfigs(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(80.0))
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(60.0))
                            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(2000)));

            setMotorOutputConfigs(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

            configMaxLinearPosition(Inches.of(54.625));
            configMaxPosition(Rotations.of(23.713));

            setAttached(false);

            // testing();

            // enableMotionMagic();
        }

        public ElevatorConfig enableMotionMagic() {
            this.motionMagicEnabled = true;
            return this;
        }
    }

    private final State state = new State();

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private ElevatorConfig config;

    private SysIdRoutine upSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(1.3), null),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    private SysIdRoutine downSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(Volts.of(0.1).per(Second), Volts.of(0.1), null),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            motor.setPosition(0);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Stats/Angle (Deg)", getPosition().in(Degrees));
        SmartDashboard.putNumber("Elevator/Stats/Height (In.)", getHeight().in(Inches));
        SmartDashboard.putBoolean("Elevator/Stats/Near Target Height", state.isNearTargetHeight());
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

    void logMotor(SysIdRoutineLog log) {
        log.motor("elevator")
                .voltage(getVoltage())
                .angularVelocity(getVelocity())
                .angularPosition(getPosition());
    }

    @Override
    protected void setVoltageOut(Voltage volts) {
        if (getHeight().gt(config.getMaxLinearPosition().minus(Inches.of(3.0)))) {
            volts = Volts.of(0.35);
        }
        if (getHeight().isNear(Inches.zero(), Inches.of(4.0))) {
            volts = Volts.of(Math.max(volts.in(Volts), 0.35));
        }
        super.setVoltageOut(volts);
    }

    public Command safelyLowerToBottom() {
        if (!isAttached()) return run(() -> {});
        return startRun(
                        () -> {
                            state.setTargetLevel(Level.Bottom);
                        },
                        () -> {
                            if (state.isNearTargetHeight()) {
                                setVoltageOut(Volts.zero());
                            } else {
                                setVoltageOut(Volts.of(0.35));
                            }
                        })
                .withName("Elevator.Safely Lower to Bottom");
    }

    public Command goToLevel(Level level) {
        if (!isAttached()) return run(() -> {});

        if (!config.isMotionMagicEnabled()) return safelyLowerToBottom();

        return startRun(
                        () -> {
                            state.setTargetLevel(level);
                        },
                        () -> {
                            if (state.isNearTargetHeight()
                                    && state.getTargetLevel().equals(Level.Bottom))
                                setVoltageOut(Volts.zero());
                            else
                                motor.setControl(
                                        motionMagicVoltage.withPosition(
                                                linearPositionToPosition(level.getHeight(config))));
                        })
                .withName("Elevator.Go To " + level.name());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        switch (direction) {
            case kForward:
                return upSysIdRoutine.quasistatic(Direction.kForward);
            case kReverse:
                return downSysIdRoutine.quasistatic(Direction.kReverse);
            default:
                return Commands.none();
        }
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        switch (direction) {
            case kForward:
                return upSysIdRoutine.dynamic(Direction.kForward);
            case kReverse:
                return downSysIdRoutine.dynamic(Direction.kReverse);
            default:
                return Commands.none();
        }
    }

    public State getState() {
        return state;
    }

    public enum Level {
        Bottom,
        L1Pre,
        L1,
        L2Pre,
        L2,
        L3Pre,
        L3,
        L4Pre,
        L4,
        PickUp;

        Distance getHeight(ElevatorConfig config) {
            switch (this) {
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
                case L1Pre:
                    return config.getL1Pre();
                case L2Pre:
                    return config.getL2Pre();
                case L3Pre:
                    return config.getL3Pre();
                case L4Pre:
                    return config.getL4Pre();
                default:
                    return Inches.of(0);
            }
        }

        Distance getReadyToDeployHeight(ElevatorConfig config) {
            switch (this) {
                case L1, L1Pre:
                    return config.getL1Pre();
                case L2, L2Pre:
                    return config.getL2Pre();
                case L3, L3Pre:
                    return config.getL3Pre();
                case L4, L4Pre:
                    return config.getL4Pre();
                default:
                    return config.getMaxLinearPosition(); // theoretically never
            }
        }
    }

    public class State {
        @Getter @Setter private boolean climbing;
        @Getter @Setter private Level targetLevel = Level.Bottom;

        public boolean isReadyToDeploy() {
            return getHeight().plus(Inches.of(0.5)).gt(targetLevel.getReadyToDeployHeight(config));
        }

        public boolean isNearTargetHeight() {
            return getHeight().isNear(targetLevel.getHeight(config), Inches.of(0.5));
        }
    }

    @Override
    public void setupBindings() {
        ElevatorStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        ElevatorStates.setupDefaultCommand();
    }
}
