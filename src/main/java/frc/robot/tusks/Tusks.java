package frc.robot.tusks;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hocLib.motionmagic.MotionMagicV5;
import java.util.function.Supplier;

public class Tusks extends SubsystemBase {

    public static class TusksConfig {}

    final State state = new State();

    final TusksMotor tusks = new TusksMotor();

    private final boolean DISABLE_MOTION_MAGIC = true;

    private TusksConfig config;

    public Tusks(TusksConfig config) {
        super();
        this.config = config;

        setDefaultCommand(Commands.either(run(tusks::zero), up(), () -> DISABLE_MOTION_MAGIC));

        SmartDashboard.putData("Tusks/Tusks", this);

        SmartDashboard.putData("Tusks/Commands/Pickup", pickup());
        SmartDashboard.putData("Tusks/Commands/Deploy", deploy());

        SmartDashboard.putNumber("Tusks/Commands/Raw Voltage Out", 0.0);
        SmartDashboard.putData(
                "Tusks/Commands/Apply Voltage Out",
                voltageOut(
                        () -> {
                            var voltage =
                                    SmartDashboard.getNumber("Tusks/Commands/Raw Voltage Out", 0.0);
                            return Volts.of(voltage);
                        }));
    }

    @Override
    public String getName() {
        return "Tusks";
    }

    @Override
    public void periodic() {
        if (!state.hasCoral() // has no coral
                && tusks.getVelocity()
                        .lt(
                                DegreesPerSecond
                                        .zero()) // tusks are moving down despite the motor moving
                // up
                && tusks.getError().gt(Degrees.of(5))) { // tusks are far below target angle
            state.setHasCoral(true); //
        }

        if (tusks.getAngle().lt(Degrees.of(-15))) {
            state.setHasCoral(false);
        }

        SmartDashboard.putNumber("Tusks/Stats/Angle (Deg.)", tusks.getAngle().in(Degrees));
        SmartDashboard.putNumber("Tusks/Stats/Output Voltage", tusks.getVoltageOutput().in(Volts));
        SmartDashboard.putBoolean("Tusks/Stats/Deploying", state.isDeploying());
        SmartDashboard.putNumber(
                "Tusks/Stats/Velocity (RPS) of Motor",
                tusks.getVelocityOfMotor().in(RotationsPerSecond));
    }

    public enum Side {
        Left,
        Right
    }

    public Command up() {
        return run(() -> tusks.setAngle(Degrees.of(90)));
    }

    public Command pickup() {
        return run(() -> tusks.setAngle(Degrees.of(20)));
    }

    public Command deploy() {
        return deploy(() -> true);
    }

    public Command deploy(Supplier<Boolean> deploy) {
        return runEnd(
                        () -> {
                            if (deploy.get()) state.setDeploying(true);

                            if (state.isDeploying()) tusks.setAngle(Degrees.of(-30));
                            else tusks.setAngle(Degrees.of(90));
                        },
                        () -> state.notDeploying())
                .until(() -> !state.hasCoral());
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> tusks.setVoltageOutput(voltsSupplier.get()));
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

        public Angle getAngle() {
            return tusks.getAngle();
        }
    }

    class TusksMotor {
        final TalonSRX motor = new TalonSRX(17);

        final ArmFeedforward ff_noCoral = new ArmFeedforward(0, 0, 0);
        final ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0);

        final double GEAR_RATIO = 100.0 / 1.0; // 100 rotations of motor is 1 rotation of tusk
        final double TICKS_PER_REV = 4096.0;

        TusksMotor() {
            var config = new TalonSRXConfiguration();

            config.motionCruiseVelocity = 2.0 * TICKS_PER_REV / 10;
            config.motionAcceleration =
                    config.motionCruiseVelocity * 2; // takes half a second to get to max velocity
            config.motionCurveStrength = 1;

            var slotConfig = new SlotConfiguration();

            slotConfig.kP = 0.0;
            slotConfig.kI = 0.0;
            slotConfig.kD = 0.0;

            slotConfig.kF = MotionMagicV5.convV6toV5(0.0, motor, TICKS_PER_REV);

            config.slot0 = slotConfig;

            var slot1Config = new SlotConfiguration();

            slot1Config.kP = 0.0;
            slot1Config.kI = 0.0;
            slot1Config.kD = 0.0;

            slot1Config.kF = MotionMagicV5.convV6toV5(0.0, motor, TICKS_PER_REV);

            config.slot1 = slot1Config;

            motor.configAllSettings(config);

            motor.setInverted(false);
            motor.setSensorPhase(false);

            motor.setSelectedSensorPosition(fromAngle(Degrees.of(90)));
        }

        double fromAngle(Angle angle) {
            return angle.times(GEAR_RATIO).in(Rotations) * TICKS_PER_REV;
        }

        Angle toAngle(double ticks) {
            return Rotations.of(ticks / TICKS_PER_REV).div(GEAR_RATIO);
        }

        Angle getAngle() {
            return toAngle(motor.getSelectedSensorPosition());
        }

        AngularVelocity getVelocity() {
            return toAngle(motor.getSelectedSensorVelocity()).div(Milliseconds.of(100));
        }

        AngularVelocity getVelocityOfMotor() {
            return getVelocity().times(GEAR_RATIO);
        }

        Angle getError() {
            return toAngle(motor.getClosedLoopTarget() - motor.getSelectedSensorPosition());
        }

        ArmFeedforward getCurrentFF() {
            return (state.hasCoral()) ? ff_withCoral : ff_noCoral;
        }

        double getArbitraryFF() {
            return getCurrentFF().calculate(getAngle().in(Radians), motor.getClosedLoopError())
                    / motor.getBusVoltage();
        }

        void setAngle(Angle angle) {
            var ticks = fromAngle(angle);
            if (!DISABLE_MOTION_MAGIC)
                motor.set(
                        ControlMode.MotionMagic,
                        ticks,
                        DemandType.ArbitraryFeedForward,
                        getArbitraryFF());
            else zero();
        }

        void zero() {
            setVoltageOutput(Volts.zero());
        }

        void setVoltageOutput(Voltage volts) {
            if (getAngle().gt(Degrees.of(90))) volts = Volts.of(Math.min(volts.in(Volts), 0.0));

            if (getAngle().lt(Degrees.of(-50))) volts = Volts.of(Math.max(volts.in(Volts), 0.0));

            motor.set(ControlMode.PercentOutput, volts.in(Volts) / motor.getBusVoltage());
        }

        Voltage getVoltageOutput() {
            return Volts.of(motor.getMotorOutputVoltage());
        }
    }
}
