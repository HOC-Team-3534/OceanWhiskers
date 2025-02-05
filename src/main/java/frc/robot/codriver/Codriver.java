package frc.robot.codriver;

import frc.hocLib.gamepads.Gamepad;

public class Codriver extends Gamepad {

    public static class CodriverConfig extends Config {
        public CodriverConfig() {
            super("Codriver", 1);
            setTriggersDeadzone(0.0);
        }
    }

    private CodriverConfig config;

    public Codriver(CodriverConfig config) {
        super(config);
        this.config = config;
    }
}
