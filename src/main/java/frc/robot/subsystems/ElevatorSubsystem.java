package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final State state = new State();

    private final Distance DEPLOY_RAISE_HEIGHT = Inches.of(5.0);

    private final boolean DISABLE_MOTION_MAGIC = true;

    private final Elevator elevator = new Elevator();

    private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(10.0))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15.0))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.of(100).per(Second));

    public ElevatorSubsystem() {
        super();

        setDefaultCommand(goToLevel(Level.Bottom));

        SmartDashboard.putData("Elevator/Elevator", this);

        SmartDashboard.putBoolean("Elevator/Commands/Deploy", false);

        Supplier<Boolean> getDeploy = () -> SmartDashboard.getBoolean("Elevator/Commands/Deploy", false);

        SmartDashboard.putData("Elevator/Commands/l1", goToLevel(Level.L1, getDeploy));
        SmartDashboard.putData("Elevator/Commands/l2", goToLevel(Level.L2, getDeploy));
        SmartDashboard.putData("Elevator/Commands/l3", goToLevel(Level.L3, getDeploy));
        SmartDashboard.putData("Elevator/Commands/l4", goToLevel(Level.L4, getDeploy));

        SmartDashboard.putData("Elevator/Commands/PickUp", goToLevel(Level.PickUp));
        SmartDashboard.putNumber("Elevator/Commands/Raw Voltage Out", 0.0);
        SmartDashboard.putData("Elevator/Commands/Apply Voltage Out", voltageOut(() -> {
            var voltage = SmartDashboard.getNumber("Elevator/Commands/Raw Voltage Out", 0.0);
            return Volts.of(voltage);
        }));

    }

    LinearFilter rpsVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Stats/Angle (Deg)", elevator.getRawPosition().in(Degrees));
        SmartDashboard.putNumber("Elevator/Stats/Height (In.)", elevator.getHeight().in(Inches));
        SmartDashboard.putNumber("Elevator/Stats/Target Height (In.)", elevator.getTargetHeight().in(Inches));
        SmartDashboard.putBoolean("Elevator/Stats/Near Target Height", elevator.isNearTargetHeight());
        SmartDashboard.putBoolean("Elevator/Stats/Deploying", state.isDeploying());
        SmartDashboard.putNumber("Elevator/Stats/Voltage Output", elevator.getVoltageOutput().in(Volts));

        rpsVelocityFilter.calculate(elevator.getRawVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Elevator/Stats/Velocity (RPS)", rpsVelocityFilter.lastValue());

        currentProfileState = currentTheorecticalProfile.calculate(0.02, currentProfileState,
                new TrapezoidProfile.State(elevator.getTargetHeight().in(Inches), 0));

        SmartDashboard.putNumber("Elevator/Stats/Theorectical Height (In.)", currentProfileState.position);
    }

    @Override
    public String getName() {
        return "Elevator";
    }

    public Command goToLevel(Level level) {
        return goToLevel(level, () -> false);
    }

    private TrapezoidProfile.State currentProfileState = new TrapezoidProfile.State();

    private TrapezoidProfile currentTheorecticalProfile = new TrapezoidProfile(
            new Constraints(elevator
                    .toHeight(RotationsPerSecond.of(mmConfigs.MotionMagicCruiseVelocity).times(Seconds.of(1)))
                    .in(Inches),
                    elevator.toHeight(
                            RotationsPerSecondPerSecond.of(mmConfigs.MotionMagicAcceleration).times(Seconds.of(1))
                                    .times(Seconds.of(1)))
                            .in(Inches)));

    public Command goToLevel(Level level, Supplier<Boolean> deploy) {
        return new FunctionalCommand(() -> {
            currentProfileState = new TrapezoidProfile.State(elevator.getHeight().in(Inches),
                    elevator.toHeight(elevator.getRawVelocity().times(Seconds.of(1))).in(Inches));
        }, () -> {
            state.setTargetLevel(level);
            switch (level) {
                case Bottom:
                    state.setDeploying(false);
                    break;
                default:
                    if (deploy.get()) {
                        state.setDeploying(true);
                    }
                    break;
            }
            elevator.updateHeight();
        }, (interrupted) -> elevator.slowlyLower(), () -> false, this).withName("Go To " + level.name());
    }

    void zero() {
        elevator.setVoltageOutput(Volts.zero());
    }

    public Command voltageOut(Supplier<Voltage> volts) {
        return run(() -> {
            getCurrentCommand().setName("Apply Voltage Out - " + elevator.getVoltageOutput().in(Volts));
            elevator.setVoltageOutput(volts.get());
        });
    }

    public State getState() {
        return state;
    }

    public enum Level {
        Bottom(Inches.of(0.0)),
        L1(Inches.of(10.0)),
        L2(Inches.of(20.0)),
        L3(Inches.of(30.0)),
        L4(Inches.of(40.0)),
        PickUp(Inches.of(15.0));

        final Distance height;

        Level(Distance height) {
            this.height = height;
        }

        Distance getHeight() {
            return height;
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
            return elevator.isNearTargetHeight();
        }
    }

    class Elevator {
        private final Angle MAX_HEIGHT_ANGLE = Rotations.of(23.713); // TODO: fix max
        private final Distance MAX_HEIGHT_LINEAR = Inches.of(54.625); // Checked with Manny. Total travel of elevator

        private final TalonFX leader = new TalonFX(14);
        private final TalonFX follower = new TalonFX(15);

        private final VoltageOut voltageOut = new VoltageOut(0);

        Elevator() {
            follower.setControl(new Follower(14, true));

            leader.setPosition(0);

            applySlot0Configs();
            applyMotionMagicConfigs();
            applyFeedbackConfigs();
            applyMotorOutputConfigs();
        }

        public Angle getRawPosition() {
            return leader.getPosition().getValue();
        }

        Distance toHeight(Angle angle) {
            return angle.div(MAX_HEIGHT_ANGLE).times(MAX_HEIGHT_LINEAR);
        }

        Angle fromHeight(Distance height) {
            return height.div(MAX_HEIGHT_LINEAR).times(MAX_HEIGHT_ANGLE);
        }

        public Distance getHeight() {
            return toHeight(getRawPosition());
        }

        public AngularVelocity getRawVelocity() {
            return leader.getVelocity().getValue();
        }

        void setVoltageOutput(Voltage volts) {
            if (getHeight().gt(MAX_HEIGHT_LINEAR.minus(Inches.of(3.0)))) {
                volts = Volts.of(0.35);
            }
            if (volts.lt(Volts.zero())) {
                volts = Volts.of(0.35);
            }
            leader.setControl(voltageOut.withOutput(volts));
        }

        Voltage getVoltageOutput() {
            return leader.getMotorVoltage().getValue();
        }

        Distance getTargetHeight() {
            return state.getTargetLevel().getHeight().plus(state.isDeploying() ? DEPLOY_RAISE_HEIGHT : Inches.zero());
        }

        Angle getTargetRawPosition() {
            var targetPosition = fromHeight(getTargetHeight());

            if (targetPosition.gt(MAX_HEIGHT_ANGLE))
                targetPosition = MAX_HEIGHT_ANGLE;
            if (targetPosition.lt(Rotations.of(0)))
                targetPosition = Rotations.of(0);

            return targetPosition;
        }

        public boolean isNearTargetHeight() {
            return elevator.getHeight().isNear(getTargetHeight(),
                    Inches.of(0.5));
        }

        public void updateHeight() {
            if (!DISABLE_MOTION_MAGIC) {
                if (elevator.isNearTargetHeight() || state.isClimbing())
                    zero();
                else
                    leader.setControl(new MotionMagicVoltage(getTargetRawPosition()));
            } else {
                if (getHeight().lt(Inches.of(1.0))) {
                    zero();
                } else {
                    slowlyLower();
                }
            }
        }

        void slowlyLower() {
            setVoltageOutput(Volts.of(0.35));
        }

        void applySlot0Configs() {
            var slotConfigs = new SlotConfigs();

            slotConfigs.kP = 0.0;
            slotConfigs.kI = 0;
            slotConfigs.kD = 0;

            slotConfigs.kG = 0.55;
            slotConfigs.kS = 0.48;
            slotConfigs.kV = 0.1086;
            slotConfigs.kA = 0;

            leader.getConfigurator().apply(slotConfigs);
        }

        void applyMotionMagicConfigs() {

            leader.getConfigurator().apply(mmConfigs);
        }

        void applyFeedbackConfigs() {
            var feedbackConfigs = new FeedbackConfigs();

            feedbackConfigs.RotorToSensorRatio = 1.0;
            feedbackConfigs.SensorToMechanismRatio = 1.0;

            leader.getConfigurator().apply(feedbackConfigs);
        }

        void applyMotorOutputConfigs() {
            var motorOutputConfig = new MotorOutputConfigs();

            motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;

            leader.getConfigurator().apply(motorOutputConfig);
        }

    }
}
