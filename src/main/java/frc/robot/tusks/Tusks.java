package frc.robot.tusks;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
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
        @Getter private boolean motionProfilingEnabled;

        @Getter private Angle up = Degrees.of(90);
        @Getter private Angle pickup = Degrees.of(30);
        @Getter private Angle preDeploy = Degrees.of(50);
        @Getter private Angle deploy = Degrees.of(-20);

        @Getter
        private double kP =
                0.001; // TODO: tune once feedforward and good motion profile created and roughly
        // following

        @Getter private double kI = 0.0;
        @Getter private double kD = 0.0;

        @Getter @Setter
        ArmFeedforward ff_noCoral =
                new ArmFeedforward(
                        0.6425,
                        0.14478,
                        5.4109 / (Math.PI * 2),
                        0.55681 / (Math.PI * 2)); // TODO: Need more testing
        // Going to fast and osciallating still... if tusks are about 14in and kg is 0.14478, then
        // kA before conversion should be 0.0217

        @Getter @Setter ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0);

        // profile in rotations while ff in radians
        @Getter @Setter
        TrapezoidProfile.Constraints profileConstants =
                new TrapezoidProfile.Constraints(0.5, 0.5); // TODO: tune along with arm feedforward

        public TusksConfig() {
            super("Tusks", 18, 1440, 1.0);

            // setAttached(false);

            // testing();

            enableMotionProfiling();
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
        return goToAngle(config.up);
    }

    public Command pickup() {
        return goToAngle(config.pickup);
    }

    public Command preDeploy() {
        return goToAngle(config.preDeploy);
    }

    public Command deploy() {
        return goToAngle(config.deploy);
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> setVoltageOut(voltsSupplier.get()));
    }

    Angle getError() {
        if (!isAttached()) return Rotations.zero();
        return positionInSensorTicksToPosition(
                motor.getClosedLoopTarget() - motor.getSelectedSensorPosition());
    }

    Command goToAngle(Angle angle) {
        if (!isAttached()) return run(() -> {});

        if (!config.isMotionProfilingEnabled()) return run(this::zero);

        return voltageOut(
                () -> {
                    profile.setGoal(angle);
                    return profile.calculate();
                });
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

    public class MotionProfileCalculator {
        final ProfiledPIDController pid;

        public MotionProfileCalculator() {
            pid =
                    new ProfiledPIDController(
                            config.kP, config.kI, config.kD, config.profileConstants);
        }

        TrapezoidProfile.State getSetpoint() {
            return pid.getSetpoint();
        }

        void setGoal(Angle angle) {
            pid.setGoal(angle.in(Rotations));
        }

        ArmFeedforward getCurrentFF() {
            return (state.isHoldingCoral()) ? config.getFf_withCoral() : config.getFf_noCoral();
        }

        Voltage getFF(AngularVelocity nextVelocity) {
            return Volts.of(
                    getCurrentFF()
                            .calculateWithVelocities(
                                    getPosition().in(Radians),
                                    getVelocity().in(RadiansPerSecond),
                                    nextVelocity.in(RadiansPerSecond)));
        }

        Voltage calculate() {
            var pidOutput = pid.calculate(getPosition().in(Rotations));
            var ff = getFF(RotationsPerSecond.of(pid.getSetpoint().velocity));
            return ff.plus(Volts.of(pidOutput));
        }
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
