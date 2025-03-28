package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.gamepads.Gamepad;

public class Codriver extends Gamepad {

    @SuppressWarnings("unused")
    private final CodriverConfig config;

    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.negate();

    public final Trigger UseSysIdOverManual = Trigger.kFalse;

    public final Trigger GoToL1Coral_A = A.and(noFn, teleop);
    public final Trigger GoToL2Coral_B = B.and(noFn, teleop);
    public final Trigger GoToL3Coral_X = X.and(noFn, teleop);
    public final Trigger GoToL4Coral_Y = Y.and(noFn, teleop);

    public final Trigger GoToL2Algae_B = B.and(fn, teleop);
    public final Trigger GoToL3Algae_X = X.and(fn, teleop);

    public final Trigger ForceForbarIn = rightBumper.and(teleop);

    public final Trigger VoltageUp_UDP = upDpad.and(teleop, UseSysIdOverManual.not());
    public final Trigger VoltageDown_DDP = downDpad.and(teleop, UseSysIdOverManual.not());

    public final Trigger QuasiasticUp_UDP = upDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger QuasiasticDown_DDP = downDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger DynamicUp_UDP = upDpad.and(fn, teleop, UseSysIdOverManual);
    public final Trigger DynamicDown_DDP = downDpad.and(fn, teleop, UseSysIdOverManual);

    public final Trigger ReleaseAlgae_LT = leftTrigger.and(fn, teleop);

    public final Trigger PreClimb_Select = select.and(teleop);
    public final Trigger Climb_Start = start.and(teleop);

    public final Trigger RawRT = rightTrigger;

    public final Trigger Deploy_LS = leftStickClick.and(teleop);

    public static class CodriverConfig extends Gamepad.Config {
        public CodriverConfig() {
            super("Codriver", 1);
            setTriggersDeadzone(0.25);
        }
    }

    public Codriver(CodriverConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
