package frc.robot.controllers;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.gamepads.Gamepad;
import frc.robot.Robot;
import lombok.Getter;
import lombok.Setter;

public class Driver extends Gamepad {

    private DriverConfig config;

    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.negate();

    public final Trigger SwerveQuasiasticForward_UDP = upDpad.and(noFn, teleop);
    public final Trigger SwerveQuasiasticBackward_DDP = downDpad.and(noFn, teleop);
    public final Trigger SwerveDynamicForward_UDP = upDpad.and(fn, teleop);
    public final Trigger SwerveDynamicBackward_DDP = downDpad.and(fn, teleop);

    public final Trigger RobotCentricForward = upDpad.and(teleop);
    public final Trigger RobotCentricBackward = downDpad.and(teleop);
    public final Trigger RobotCentricLeft = leftDpad.and(teleop);
    public final Trigger RobotCentricRight = rightDpad.and(teleop);

    public final Trigger DTMToReefLeft_LT = leftTrigger.and(noFn, teleop);
    public final Trigger DTMToReefRight_RT = rightTrigger.and(noFn, teleop);
    public final Trigger DTMToHumanPlayerStation_B = B.and(noFn, teleop);

    public static class DriverConfig extends Config {
        @Getter @Setter private double creepModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.75;
        @Getter @Setter private double turboModeScalor = 1;

        public DriverConfig() {
            super("Driver", 0);

            setLeftStickDeadzone(0.35);
            setLeftStickExp(2.0);
            setLeftStickScalor(1);

            setRightStickDeadzone(0.2);
            setRightStickExp(2.0);
            setRightStickScalor(1);

            setTriggersDeadzone(0.25);
            setTriggersExp(1);
            setTriggersScalor(1);
        }
    }

    private Trigger CreepMode =
            (rightBumper.or(() -> Robot.getElevator().getHeight().gt(Inches.of(25))));

    @Getter @Setter
    private boolean isTurboMode = false; // TODO: change slow and turbo to RobotStates

    public Driver(DriverConfig config) {
        super(config);
        this.config = config;
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(getLeftY());
        if (CreepMode.getAsBoolean()) {
            fwdPositive *= Math.abs(config.getCreepModeScalor());
        }
        return -fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = leftStickCurve.calculate(getLeftX());
        if (CreepMode.getAsBoolean()) {
            leftPositive *= Math.abs(config.getCreepModeScalor());
        }
        return -leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (CreepMode.getAsBoolean()) {
            ccwPositive *= Math.abs(config.getCreepModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return -ccwPositive; // invert the value
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
