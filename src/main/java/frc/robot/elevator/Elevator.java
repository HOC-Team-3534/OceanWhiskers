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
        // TODO: tune heights for each level and pickup height
        @Getter private Distance L1 = Inches.of(0);
        @Getter private Distance L1Pre = Inches.of(0);
        @Getter private Distance L2 = Inches.of(14.5);
        @Getter private Distance L2Pre = Inches.of(9.5);
        @Getter private Distance L3 = Inches.of(30.5);
        @Getter private Distance L3Pre = Inches.of(25.5);
        @Getter private Distance L4 = Inches.of(54);
        @Getter private Distance L4Pre = Inches.of(38);
        @Getter private Distance PickUp = Inches.of(15);

        @Getter private boolean motionMagicEnabled;

        public ElevatorConfig() {
            super("Elevator Leader", 14);
            setFollowerConfigs(new FollowerConfig("Elevator Follower", 15, true));

            var slot0Configs = new Slot0Configs();

            slot0Configs.kP = 3.0;
            slot0Configs.kI = 0;
            slot0Configs.kD = 0;

            // spotless:off
            //https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=83&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A18%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A3.66%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A54%2C%22u%22%3A%22in%22%7D
            //spotless:on

            slot0Configs.kG = 0.805;
            slot0Configs.kS = 0.0559;
            slot0Configs.kV = 0.11793;
            slot0Configs.kA = 0.0048; // recalc says 0.007 V * s^2 / rot

            setSlot0Configs(slot0Configs);

            setMMConfigs(
                    new MotionMagicConfigs()
                            // theoretical max of 66.5 rot of motor per second
                            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(37.0))
                            // recalc says acceleration can be 99 rot per s^2
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80.0))
                            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(2000)));

            setMotorOutputConfigs(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

            configMaxLinearPosition(Inches.of(54.625));
            configMaxPosition(Rotations.of(23.713));

            // TODO: set elevator in brake mode, hoping it causes the elevator not to fall when
            // disabled, important between autonomous and teleop

            enableMotionMagic();
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
                    new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3.0), null),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    private SysIdRoutine downSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(0.5), null),
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
        if (!isAttached()) return doNothing();
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

    Command doNothing() {
        return run(() -> {}).withName("Elevator.Do Nothing");
    }

    public Command goToLevel(Level level) {
        if (!isAttached()) return doNothing();

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
                return upSysIdRoutine
                        .quasistatic(Direction.kForward)
                        .withName("Elevator.Quas Forward");
            case kReverse:
                return downSysIdRoutine
                        .quasistatic(Direction.kReverse)
                        .withName("Elevator.Quas Reverse");
            default:
                return Commands.none();
        }
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        switch (direction) {
            case kForward:
                return upSysIdRoutine.dynamic(Direction.kForward).withName("Elevator.Dyn Forward");
            case kReverse:
                return downSysIdRoutine
                        .dynamic(Direction.kReverse)
                        .withName("Elevator.Dyn Reverse");
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
