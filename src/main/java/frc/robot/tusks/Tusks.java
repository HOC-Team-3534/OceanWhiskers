package frc.robot.tusks;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.hocLib.util.CachedValue;
import frc.robot.Robot;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Tusks extends TalonSRXMechanism {

    public static class TusksConfig extends TalonSRXMechanism.Config {
        @Getter private boolean motionProfilingEnabled;

        @Getter private Angle up = Degrees.of(91);
        @Getter private Angle pickup = Degrees.of(40);
        @Getter private Angle preDeploy = Degrees.of(40);
        @Getter private Angle l4Deploy = Degrees.of(-45);
        @Getter private Angle l2l3Deploy = Degrees.of(-35);
        @Getter private Angle l1Deploy = Degrees.of(-30);

        @Getter private double kP = 1.0;

        @Getter private double kI = 0.0;
        @Getter private double kD = 0.0;

        // spotless:off
        //https://www.reca.lc/arm?armMass=%7B%22s%22%3A2%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A6%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22BAG%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        //spotless:on

        // tuned ff with coral without sysid, just manual kg and ks then recalc for kV and kA
        // kg = (voltageToMoveUp + voltageToMoveDown) / 2
        // ks = (voltageToMoveUp - voltageToMoveDown) / 2

        @Getter @Setter ArmFeedforward ff_noCoral = new ArmFeedforward(0.74, 0.16, 0.87, 0.002);

        @Getter @Setter ArmFeedforward ff_withCoral = new ArmFeedforward(0.44, 0.75, 0.87, 0.01);

        // profile in rotations while ff in radians
        @Getter @Setter
        TrapezoidProfile.Constraints profileConstants =
                new TrapezoidProfile.Constraints(
                        RotationsPerSecond.of(1.0).in(RadiansPerSecond),
                        RotationsPerSecondPerSecond.of(4.0).in(RadiansPerSecondPerSecond));

        public TusksConfig() {
            super("Tusks", 18, 1440, 1.0);
        }

        public TusksConfig enableMotionProfiling() {
            this.motionProfilingEnabled = true;
            return this;
        }
    }

    private SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(null, Volts.of(2), null),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    final State state = new State();

    final MotionProfileCalculator profile;

    private TusksConfig config;

    public Tusks(TusksConfig config) {
        super(config);
        this.config = config;

        profile = new MotionProfileCalculator();

        if (isAttached()) {

            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            motor.setSensorPhase(true);

            Timer.delay(0.02);

            motor.setSelectedSensorPosition(positionToPositionInSensorTicks(Degrees.of(90)));
        }

        SmartDashboard.putNumber("Tusks/Voltage Up Command", 0.75);
        SmartDashboard.putNumber("Tusks/Voltage Down Command", -0.75);

        SmartDashboard.putBoolean("Tusks Has Coral Override", false);
    }

    private Timer stillTimer = new Timer();

    @Override
    public void periodic() {
        if (profile.getGoal().gt(Degrees.of(80)) && getVelocity().lt(DegreesPerSecond.of(0.5))) {
            if (stillTimer.hasElapsed(0.25)) {
                if (getPosition().lt(Degrees.of(88.0))) {
                    state.setHoldingCoral(true);
                } else {
                    state.setHoldingCoral(false);
                }
            }
        } else {
            stillTimer.restart();
        }

        SmartDashboard.putNumber("Tusks/Angle (Deg.)", getPosition().in(Degrees));
        SmartDashboard.putNumber("Tusks/Output Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Tusks/Velocity (RPS)", getVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Tusks/Setpoint (Rots)", profile.getSetpoint().position);
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
        // TODO: consider continuing to drive the motor a little to hold up
        // TODO: could reset position to 90 if greater than 90
        return goToAngle(config.up);
    }

    public Command pickup() {
        return goToAngle(config.pickup)
                .alongWith(
                        Commands.run(
                                () -> {
                                    if (getVelocity().lt(DegreesPerSecond.zero())
                                            && getError().gt(Degrees.of(3.0))) {
                                        state.setHoldingCoral(true);
                                    }
                                }));
    }

    public Command preDeploy() {
        return goToAngle(config.preDeploy);
    }

    public Command deployl4() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l4Deploy, Degrees.of(5.0)))
                        .andThen(Commands.waitSeconds(0.5)),
                goToAngle(config.l4Deploy),
                Commands.startEnd(
                        () -> profile.setForceNoCoralFF(true),
                        () -> {
                            state.setHoldingCoral(false);
                            profile.setForceNoCoralFF(false);
                        }));
    }

    public Command deployl2l3() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l2l3Deploy, Degrees.of(5.0)))
                        .andThen(Commands.waitSeconds(0.25)),
                goToAngle(config.l2l3Deploy),
                Commands.startEnd(
                        () -> profile.setForceNoCoralFF(true),
                        () -> {
                            state.setHoldingCoral(false);
                            profile.setForceNoCoralFF(false);
                        }));
    }

    public Command deployl1() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l1Deploy, Degrees.of(5.0)))
                        .andThen(Commands.waitSeconds(0.25)),
                goToAngle(config.l1Deploy),
                Commands.startEnd(() -> {}, () -> state.setHoldingCoral(false)));
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> setVoltageOut(voltsSupplier.get()));
    }

    Angle getError() {
        if (!isAttached()) return Rotations.zero();
        return profile.getGoal().minus(getPosition());
    }

    Command goToAngle(Angle angle) {
        if (!isAttached()) return run(() -> {});

        if (!config.isMotionProfilingEnabled()) return run(() -> setVoltageOut(Volts.of(0.5)));

        return new InstantCommand(
                        () -> {
                            profile.reset();
                            profile.setGoal(angle);
                        })
                .alongWith(voltageOut(() -> profile.calculate()))
                .withName("Tusks.Go To " + angle.in(Degrees) + "°");
    }

    protected void zero() {
        setVoltageOut(Volts.zero());
    }

    @Override
    protected void setVoltageOut(Voltage voltage) {
        if (getPosition().gt(Degrees.of(90))) voltage = Volts.of(Math.min(voltage.in(Volts), 0.0));
        if (getPosition().lt(Degrees.of(-75))) voltage = Volts.of(Math.max(voltage.in(Volts), 0.0));
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

    public class MotionProfileCalculator {
        final ProfiledPIDController pid;
        @Getter @Setter private boolean forceNoCoralFF;

        public MotionProfileCalculator() {
            pid =
                    new ProfiledPIDController(
                            config.kP, config.kI, config.kD, config.profileConstants);
        }

        TrapezoidProfile.State getSetpoint() {
            return pid.getSetpoint();
        }

        void reset() {
            pid.reset(
                    new TrapezoidProfile.State(
                            getPosition().in(Radians), getVelocity().in(RadiansPerSecond)));
        }

        void setGoal(Angle angle) {
            pid.setGoal(angle.in(Radians));
        }

        Angle getGoal() {
            return Radians.of(pid.getGoal().position);
        }

        private ArmFeedforward prev_ff = config.getFf_withCoral();

        ArmFeedforward getCurrentFF() {
            var ff = (!state.isHoldingCoral()) ? config.getFf_noCoral() : config.getFf_withCoral();
            if (prev_ff != ff) {
                var goal = profile.getGoal();
                profile.reset();
                profile.setGoal(goal);
                prev_ff = ff;
            }
            return ff;
        }

        Voltage getFF() {
            var position = pid.getSetpoint().position;
            var velocity = pid.getSetpoint().velocity;
            // TODO: try JNI now that infinite loop should be fixed in wpilib 2025.3.1
            return Volts.of(
                    getCurrentFF()
                            .calculate(
                                    position,
                                    velocity,
                                    velocity - getVelocity().in(RadiansPerSecond)));
        }

        Voltage calculate() {
            var pidOutput =
                    pid.calculate(getPosition().in(Radians)) * (state.isHoldingCoral() ? 1 : 1);
            var ff = getFF();
            return ff.plus(Volts.of(pidOutput));
        }
    }

    public class State {
        @Setter private boolean holdingCoral;

        private CachedValue<Boolean> cachedSimulationHoldingCoralOverride =
                new CachedValue<>(
                        () -> SmartDashboard.getBoolean("Tusks Has Coral Override", false));

        public boolean isReadyToDeploy() {
            return getPosition().minus(Degrees.of(6.0)).lt(config.getPreDeploy());
        }

        public boolean isHoldingCoral() {
            return Robot.isReal() ? holdingCoral : cachedSimulationHoldingCoralOverride.get();
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
