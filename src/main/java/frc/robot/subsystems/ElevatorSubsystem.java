package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final int LEADER_ID = 14;
    private final int FOLLOWER_ID = 15;

    private final TalonFX elevator = new TalonFX(LEADER_ID);
    private final TalonFX follower = new TalonFX(FOLLOWER_ID);

    private final Angle MAX_HEIGHT_ANGLE = Rotations.of(23.713); // TODO: fix max
    private final Distance MAX_HEIGHT_LINEAR = Inches.of(54.625); // Checked with Manny. Total travel of elevator

    private final State state = new State();

    private final Distance DEPLOY_RAISE_HEIGHT = Inches.of(5.0);

    private final boolean DISABLE_MOTION_MAGIC = true;

    private final VoltageOut voltageOut = new VoltageOut(0);

    public ElevatorSubsystem() {
        super();

        follower.setControl(new Follower(LEADER_ID, true));

        elevator.setPosition(0);

        var slotConfigs = new SlotConfigs();

        slotConfigs.kP = 0.25;
        slotConfigs.kI = 0;
        slotConfigs.kD = 0;

        slotConfigs.kG = 0;
        slotConfigs.kS = 0;
        slotConfigs.kV = 0;
        slotConfigs.kA = 0;

        elevator.getConfigurator().apply(slotConfigs);

        var mmConfigs = new MotionMagicConfigs();

        mmConfigs.MotionMagicCruiseVelocity = 100;
        mmConfigs.MotionMagicAcceleration = 300;
        mmConfigs.MotionMagicJerk = 2500;

        elevator.getConfigurator().apply(mmConfigs);

        var feedbackConfigs = new FeedbackConfigs();

        feedbackConfigs.RotorToSensorRatio = 1.0;
        feedbackConfigs.SensorToMechanismRatio = 1.0;

        elevator.getConfigurator().apply(feedbackConfigs);

        var motorOutputConfig = new MotorOutputConfigs();

        motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;

        elevator.getConfigurator().apply(motorOutputConfig);

        setDefaultCommand(powerDownwardsToZero());

        SmartDashboard.putData("Elevator/Elevator", this);

        SmartDashboard.putBoolean("Elevator/Commands/Deploy", false);

        Supplier<Boolean> getDeploy = () -> SmartDashboard.getBoolean("Elevator/Commands/Deploy", false);

        SmartDashboard.putData("Elevator/Commands/l1", l1(getDeploy));
        SmartDashboard.putData("Elevator/Commands/l2", l2(getDeploy));
        SmartDashboard.putData("Elevator/Commands/l3", l3(getDeploy));
        SmartDashboard.putData("Elevator/Commands/l4", l4(getDeploy));

        SmartDashboard.putData("Elevator/Commands/PickUp", pickUp());
        SmartDashboard.putNumber("Elevator/Commands/Raw Voltage Out", 0.0);
        SmartDashboard.putData("Elevator/Commands/Apply Voltage Out", voltageOut(() -> {
            var voltage = SmartDashboard.getNumber("Elevator/Commands/Raw Voltage Out", 0.0);
            return Volts.of(voltage);
        }));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Stats/Angle (Deg)", elevator.getPosition().getValue().in(Degrees));
        SmartDashboard.putNumber("Elevator/Stats/Height (In.)", getHeight().in(Inches));
        SmartDashboard.putNumber("Elevator/Stats/Target Height (In.)", state.getTargetHeight().in(Inches));
        SmartDashboard.putBoolean("Elevator/Stats/Near Target Height", state.isNearTargetHeight());
        SmartDashboard.putBoolean("Elevator/Stats/Deploying", state.isDeploying());
        SmartDashboard.putNumber("Elevator/Stats/Voltage Output", elevator.getMotorVoltage().getValue().in(Volts));
        SmartDashboard.putNumber("Elevator/Stats/Velocity (RPS)", getVelocity().in(RotationsPerSecond));
    }

    @Override
    public String getName() {
        return "Elevator";
    }

    public Command raiseToHeight(TargetHeight targetHeight) {
        return raiseToHeight(targetHeight, () -> false);
    }

    public Command raiseToHeight(TargetHeight targetHeight, Supplier<Boolean> deploy) {
        return runEnd(() -> setHeight(targetHeight, deploy), this::setVoltageOutToZero);
    }

    Command cutPower() {
        return run(() -> setVoltageOutToZero());
    }

    Command powerDownwardsToZero() {
        return runEnd(() -> {
            state.notDeploying();
            state.setTargetHeight(TargetHeight.Bottom);
            if (!DISABLE_MOTION_MAGIC) {
                if (state.isNearTargetHeight() || state.isClimbing())
                    setVoltageOutToZero();
                else
                    setHeight(TargetHeight.Bottom);
            } else {
                if (getHeight().lt(Inches.of(2.0))) {
                    setVoltageOutToZero();
                } else {
                    elevator.setControl(voltageOut.withOutput(Volts.of(0.35)));
                }
            }
        }, this::setVoltageOutToZero).withName("Elevator Go Down");

    }

    public Command l1() {
        return l1(() -> false);
    }

    public Command l1(Supplier<Boolean> deploy) {
        return raiseToHeight(TargetHeight.L1, deploy);
    }

    public Command l2() {
        return l2(() -> false);
    }

    public Command l2(Supplier<Boolean> deploy) {
        return raiseToHeight(TargetHeight.L2, deploy);
    }

    public Command l3() {
        return l3(() -> false);
    }

    public Command l3(Supplier<Boolean> deploy) {
        return raiseToHeight(TargetHeight.L3, deploy);
    }

    public Command l4() {
        return l4(() -> false);
    }

    public Command l4(Supplier<Boolean> deploy) {
        return raiseToHeight(TargetHeight.L4, deploy);
    }

    public Command pickUp() {
        return raiseToHeight(TargetHeight.PickUp).withName("Pickup");
    }

    public Command voltageOut(Supplier<Voltage> volts) {

        return run(() -> {
            if (getHeight().isNear(MAX_HEIGHT_LINEAR, Inches.of(3.0))) {
                elevator.setControl(voltageOut.withOutput(0.35));
            } else {
                elevator.setControl(voltageOut.withOutput(volts.get()));
            }
            getCurrentCommand().setName("Apply Voltage Out - " + voltageOut.getOutputMeasure().in(Volts));
        });
    }

    Distance toHeight(Angle angle) {
        return angle.div(MAX_HEIGHT_ANGLE).times(MAX_HEIGHT_LINEAR);
    }

    Angle fromHeight(Distance height) {
        return height.div(MAX_HEIGHT_LINEAR).times(MAX_HEIGHT_ANGLE);
    }

    public Distance getHeight() {
        return toHeight(elevator.getPosition().getValue());
    }

    public AngularVelocity getVelocity() {
        return elevator.getVelocity().getValue();
    }

    public void setHeight(TargetHeight targetHeight) {
        setHeight(targetHeight, () -> false);
    }

    public void setHeight(TargetHeight targetHeight, Supplier<Boolean> deploy) {
        state.setTargetHeight(targetHeight, deploy);
        var targetPosition = fromHeight(state.getTargetHeight());

        if (targetPosition.gt(MAX_HEIGHT_ANGLE))
            targetPosition = MAX_HEIGHT_ANGLE;
        if (targetPosition.lt(Rotations.of(0)))
            targetPosition = Rotations.of(0);
        if (!DISABLE_MOTION_MAGIC)
            elevator.setControl(new MotionMagicVoltage(targetPosition));
        else
            setVoltageOutToZero();
    }

    public void setVoltageOutToZero() {
        elevator.setControl(voltageOut.withOutput(Volts.zero()));
    }

    public State getState() {
        return state;
    }

    enum TargetHeight {
        Bottom(Inches.of(0.0)),
        L1(Inches.of(10.0)),
        L2(Inches.of(20.0)),
        L3(Inches.of(30.0)),
        L4(Inches.of(40.0)),
        PickUp(Inches.of(15.0));

        final Distance height;

        TargetHeight(Distance height) {
            this.height = height;
        }

        Distance getHeight() {
            return height;
        }
    }

    public class State {
        private boolean climbing;
        private TargetHeight targetHeight = TargetHeight.Bottom;
        private boolean deploying;

        public boolean isClimbing() {
            return climbing;
        }

        void setTargetHeight(TargetHeight targetHeight) {
            setTargetHeight(targetHeight, () -> false);
        }

        void setTargetHeight(TargetHeight targetHeight, Supplier<Boolean> deploy) {
            this.targetHeight = targetHeight;

            if (deploy.get()) {
                this.deploying = true;
            }
        }

        boolean isDeploying() {
            return deploying;
        }

        void notDeploying() {
            deploying = false;
        }

        public TargetHeight getSelectedTargetHeight() {
            return targetHeight;
        }

        public boolean isReefTargetHeight() {
            switch (targetHeight) {
                case L1, L2, L3, L4:
                    return true;
                default:
                    return false;
            }
        }

        Distance getTargetHeight() {
            return targetHeight.getHeight().plus(deploying ? DEPLOY_RAISE_HEIGHT : Inches.zero());
        }

        public boolean isNearTargetHeight() {
            return getHeight().isNear(getTargetHeight(),
                    Inches.of(1.0));
        }
    }
}
