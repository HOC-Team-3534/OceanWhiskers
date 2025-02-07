package frc.robot.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

public class Driver extends Gamepad {

    private DriverConfig config;

    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.negate();

    public final Trigger SwerveQuasiasticForward_UDP = upDpad.and(noFn, teleop);
    public final Trigger SwerveQuasiasticBackward_DDP = downDpad.and(noFn, teleop);
    public final Trigger SwerveDynamicForward_UDP = rightDpad.and(fn, teleop);
    public final Trigger SwerveDynamicBackward_DDP = leftDpad.and(fn, teleop);

    public static class DriverConfig extends Config {
        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.75;
        @Getter @Setter private double turboModeScalor = 1;
        private double deadzone = 0.001;

        public DriverConfig() {
            super("Driver", 0);
            setTriggersDeadzone(0.0);
            setLeftStickDeadzone(deadzone);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setRightStickDeadzone(deadzone);
            setRightStickExp(2.0);
            setRightStickScalor(12);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalor(1);
        }
    }

    @Getter @Setter
    private boolean isSlowMode = false; // TODO: change slow and turbo to RobotStates

    @Getter @Setter private boolean isTurboMode = false;

    public Driver(DriverConfig config) {
        super(config);
        this.config = config;
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return -1 * ccwPositive; // invert the value
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
