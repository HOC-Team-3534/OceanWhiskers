package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonFXMechanism;
import frc.robot.Robot;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends TalonFXMechanism {

    @Getter
    public static class ElevatorConfig extends Config {
        Distance L1 = Inches.of(0);
        Distance L1Pre = Inches.of(0);
        Distance L2 = Inches.of(14.5);
        Distance L2Pre = Inches.of(3.5);
        Distance L3 = Inches.of(30.5);
        Distance L3Pre = Inches.of(19.5);
        Distance L4 = Inches.of(54.5);
        Distance L4Pre = Inches.of(54.5);
        Distance PickUp = Inches.of(0.0);
        // TODO: tune jaw height
        Distance Jaws = Inches.of(18.0);
        Distance PreClimb = Inches.of(15);
        // TODO: tune algae heights
        Distance L2Algae = Inches.of(23.0);
        Distance L3Algae = Inches.of(40.0);

        boolean motionMagicEnabled;

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

            slot0Configs.kG = 0.975;
            slot0Configs.kS = 0.425;
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

    private ElevatorSim elevatorSim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            Timer.delay(0.100);
            motor.setPosition(0);
            motor.setNeutralMode(NeutralModeValue.Brake);
            for (var follower : followerMotors) {
                follower.setNeutralMode(NeutralModeValue.Brake);
            }

            if (RobotBase.isSimulation()) {
                elevatorSim =
                        new ElevatorSim(
                                DCMotor.getFalcon500(2),
                                5,
                                Pounds.of(30).in(Kilograms),
                                Inches.of(1.83).in(Meters),
                                0.0,
                                config.getMaxLinearPosition().in(Meters),
                                true,
                                0.0,
                                0.01,
                                0.0);
            }
        }

        SmartDashboard.putNumber("Elevator/Voltage Up Command", 1.55);
        SmartDashboard.putNumber("Elevator/Voltage Down Command", 0.35);
    }

    @Override
    protected Voltage updateVoltage() {
        if (RobotBase.isSimulation()) return Volts.of(-motor.getSimState().getMotorVoltage());
        return super.updateVoltage();
    }

    @Override
    protected Voltage updateSupplyVoltage() {
        if (RobotBase.isSimulation()) return Volts.of(12.0);
        return super.updateSupplyVoltage();
    }

    @Override
    protected Current updateCurrent() {
        if (RobotBase.isSimulation()) return Amps.of(elevatorSim.getCurrentDrawAmps());
        return super.updateCurrent();
    }

    @Override
    protected Angle updatePosition() {
        if (RobotBase.isSimulation())
            return linearPositionToPosition(Meters.of(elevatorSim.getPositionMeters()));
        return super.updatePosition();
    }

    @Override
    protected AngularVelocity updateVelocity() {
        if (RobotBase.isSimulation())
            return linearPositionToPosition(Meters.of(elevatorSim.getVelocityMetersPerSecond()))
                    .per(Second);
        return super.updateVelocity();
    }

    @Override
    protected Double updatePercentage() {
        return super.updatePercentage();
    }

    @Override
    protected Distance updateLinearPosition() {
        if (RobotBase.isSimulation()) return Meters.of(elevatorSim.getPositionMeters());
        return super.updateLinearPosition();
    }

    @Override
    public void periodic() {
        Logging.log("Elevator", this);
        Logging.log("Elevator/Near Target Height", state.isNearTargetHeight());
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            double motorVoltage = -motor.getSimState().getMotorVoltage();
            elevatorSim.setInput(motorVoltage);

            elevatorSim.update(Robot.getConfig().LoopPeriod.in(Seconds));

            motor.getSimState().setRawRotorPosition(updatePosition());
            motor.getSimState().setRotorVelocity(updateVelocity());
        }
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
        if (getHeight().isNear(Inches.zero(), Inches.of(4.0)) && !state.isClimbing()) {
            volts = Volts.of(Math.max(volts.in(Volts), 0.0));
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
                                // setVoltageOut(Volts.of(0.35));
                                setVoltageOut(Volts.of(0.6));
                            }
                        })
                .withName("Elevator.Safely Lower to Bottom");
    }

    Command doNothing() {
        return run(() -> {}).withName("Elevator.Do Nothing");
    }

    public Command goToLevel(Supplier<Level> levelSupplier) {
        if (!isAttached()) return doNothing();

        if (!config.isMotionMagicEnabled()) return safelyLowerToBottom();

        return Commands.either(
                        run(
                                () -> {
                                    if (Robot.getTusks().getPosition().gt(Degrees.of(10))) {
                                        var level = levelSupplier.get();
                                        state.setTargetLevel(level);
                                        if (state.isNearTargetHeight()
                                                && state.getTargetLevel()
                                                        .getHeight(config)
                                                        .lt(Inches.of(0.5)))
                                            setVoltageOut(Volts.zero());
                                        else
                                            motor.setControl(
                                                    motionMagicVoltage.withPosition(
                                                            linearPositionToPosition(
                                                                    level.getHeight(config))));
                                    }
                                }),
                        run(() -> setVoltageOut(Volts.zero())),
                        () -> !state.isClimbing())
                .alongWith(
                        Commands.run(
                                () ->
                                        this.getCurrentCommand()
                                                .setName(
                                                        "Elevator.Go To "
                                                                + state.getTargetLevel())));
    }

    // TODO: add climb command and climb level for pre climb, then lockout go to height once
    // climbing occurs
    // Don't let climbing occur unless the current go to height is already pre climb, or has been
    // pre climb within the last say 5 seconds

    public Command climb() {
        return startRun(() -> state.setClimbing(true), () -> setVoltageOut(Volts.of(0)))
                .until(() -> getHeight().lt(Inches.of(15.0)))
                .andThen(
                        runEnd(
                                () -> setVoltageOut(Volts.of(-5.5)),
                                () -> setVoltageOut(Volts.zero())));
    }

    // TODO: add heights for coral deploy, make codriver fn switch between coral and algae height

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

    public Distance getTargetHeight() {
        return getState().getTargetLevel().getHeight(config);
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
        PickUp,
        Jaws,
        PreClimb,
        L2Algae,
        L3Algae;

        public Distance getHeight(ElevatorConfig config) {
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
                case Jaws:
                    return config.getJaws();
                case PreClimb:
                    return config.getPreClimb();
                case L2Algae:
                    return config.getL2Algae();
                case L3Algae:
                    return config.getL3Algae();
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

        public Pose3d getStage1Displacement() {
            return new Pose3d(
                    new Translation3d(0.0, 0.0, getHeight().div(2).in(Meters)), new Rotation3d());
        }

        public Pose3d getStage2Displacement() {
            return new Pose3d(
                    new Translation3d(0.0, 0.0, getHeight().in(Meters)), new Rotation3d());
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

    @Override
    public void disabledInit() {
        setVoltageOut(Volts.zero());
    }
}
