package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TusksSubsystem extends SubsystemBase {
    final TalonSRX tusks = new TalonSRX(17);
    final DigitalInput leftSwitch = new DigitalInput(1);
    final DigitalInput rightSwitch = new DigitalInput(2);

    final ArmFeedforward ff_noCoral = new ArmFeedforward(0, 0, 0);
    final ArmFeedforward ff_withCoral = new ArmFeedforward(0.0, 0.0, 0.0);

    final double GEAR_RATIO = 45.0 / 1.0; // 45 rotations of motor is 1 rotation of tusk
    final double TICKS_PER_REV = 4096.0;

    final State state = new State();

    public TusksSubsystem() {
        super();

        setDefaultCommand(off());

        var config = new TalonSRXConfiguration();

        config.motionCruiseVelocity = 0.0;
        config.motionAcceleration = 0.0;
        config.motionCurveStrength = 1;

        var slotConfig = new SlotConfiguration();

        slotConfig.kP = 0.0;
        slotConfig.kI = 0.0;
        slotConfig.kD = 0.0;

        config.slot0 = slotConfig;

        tusks.configAllSettings(config);

        tusks.setInverted(false);
        tusks.setSensorPhase(false);

        tusks.setSelectedSensorPosition(fromAngle(Degrees.of(90)));
    }

    @Override
    public void periodic() {
        if (leftSwitch.get()) {
            state.setLeft();
        }

        if (rightSwitch.get()) {
            state.setRight();
        }

        if (getAngle().lt(Degrees.zero()) && !leftSwitch.get() && !rightSwitch.get()) {
            if (!state.getiTimer().isRunning())
                state.getiTimer().restart();
        } else {
            state.getiTimer().stop();

        }

        if (state.getiTimer().hasElapsed(0.25)) {
            state.reset();
        }

    }

    double fromAngle(Angle angle) {
        return angle.times(GEAR_RATIO).in(Rotations) * TICKS_PER_REV;

    }

    Angle toAngle(double ticks) {
        return Rotations.of(ticks / TICKS_PER_REV).div(GEAR_RATIO);
    }

    Angle getAngle() {
        return toAngle(tusks.getSelectedSensorPosition());
    }

    AngularVelocity getVelocity() {
        return toAngle(tusks.getSelectedSensorVelocity()).div(Milliseconds.of(100));
    }

    ArmFeedforward getCurrantFF() {
        return (state.isOnLeft() || state.isOnRight()) ? ff_withCoral : ff_noCoral;
    }

    void setAngle(Angle angle) {
        var ticks = fromAngle(angle);
        tusks.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward,
                getCurrantFF().calculate(getAngle().in(Radians), getVelocity().in(RadiansPerSecond)));

    }

    public Command off() {
        return run(() -> tusks.set(ControlMode.PercentOutput, 0));
    }

    public Command up() {
        return run(() -> setAngle(Degrees.of(90)));
    }

    public Command pickup() {
        return run(() -> setAngle(Degrees.of(20)));
    }

    public Command deploy() {
        return run(() -> setAngle(Degrees.of(-30))).until(() -> !state.isOnLeft() && !state.isOnRight());
    }

    public class State {
        boolean hasCoralOnLeft;
        boolean hasCoralOnRight;
        Timer angleDownTimer = new Timer();

        void reset() {
            hasCoralOnLeft = false;
            hasCoralOnRight = false;
        }

        boolean isOnLeft() {
            return hasCoralOnLeft;
        }

        boolean isOnRight() {
            return hasCoralOnRight;
        }

        void setLeft() {
            hasCoralOnLeft = true;
        }

        void setRight() {
            hasCoralOnRight = true;
        }

        Timer getiTimer() {
            return angleDownTimer;
        }
    }
}
