package frc.robot.codriver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.gamepads.Gamepad;

public class Codriver extends Gamepad {

    private final CodriverConfig config;

    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.negate();

    public final Trigger UseSysIdOverManual = new Trigger(() -> true);

    public final Trigger GoToL1_A = A.and(noFn, teleop);
    public final Trigger GoToL2_B = B.and(noFn, teleop);
    public final Trigger GoToL3_X = X.and(noFn, teleop);
    public final Trigger GoToL4_Y = Y.and(noFn, teleop);

    public final Trigger ElevatorVoltageUp_UDP = upDpad.and(teleop, UseSysIdOverManual.not());
    public final Trigger ElevatorVoltageDown_DDP = downDpad.and(teleop, UseSysIdOverManual.not());

    public final Trigger ElevatorQuasiasticUp_UDP = upDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger ElevatorQuasiasticDown_DDP =
            downDpad.and(noFn, teleop, UseSysIdOverManual);
    public final Trigger ElevatorDynamicUp_UDP = upDpad.and(fn, teleop, UseSysIdOverManual);
    public final Trigger ElevatorDynamicDown_DDP = downDpad.and(fn, teleop, UseSysIdOverManual);

    public static class CodriverConfig extends Gamepad.Config {
        public CodriverConfig() {
            super("Codriver", 1);
            setTriggersDeadzone(0.0);
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
