package frc.robot.codriver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.gamepads.Gamepad;

public class Codriver extends Gamepad {

    private final CodriverConfig config;

    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.negate();

    public final Trigger UseSysIdOverManual = Trigger.kTrue;

    public final Trigger GoToL1_A = A.and(teleop);
    public final Trigger GoToL2_B = B.and(teleop);
    public final Trigger GoToL3_X = X.and(teleop);
    public final Trigger GoToL4_Y = Y.and(teleop);

    public final Trigger VoltageUp_UDP = upDpad.and(teleop, UseSysIdOverManual.not());
    public final Trigger VoltageDown_DDP = downDpad.and(teleop, UseSysIdOverManual.not());

    public final Trigger QuasiasticUp_UDP = upDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger QuasiasticDown_DDP = downDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger DynamicUp_UDP = upDpad.and(fn, teleop, UseSysIdOverManual);
    public final Trigger DynamicDown_DDP = downDpad.and(fn, teleop, UseSysIdOverManual);

    public final Trigger GrabAlgae_RT = rightTrigger.and(fn, teleop);
    public final Trigger ReleaseAlgae_LT = leftTrigger.and(fn, teleop);

    public final Trigger PickupCoralLeft_LT = leftTrigger.and(noFn, teleop);
    public final Trigger PickupCoralRight_RT = rightTrigger.and(noFn, teleop);

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
