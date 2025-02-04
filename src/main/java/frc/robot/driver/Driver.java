package frc.robot.driver;

import frc.hocLib.gamepads.Gamepad;

public class Driver extends Gamepad {

    public static class DriverConfig extends Config {
        public DriverConfig() {
            super("Driver", 0);
            setTriggersDeadzone(0.0);
        }
    }

    private DriverConfig config;

    public Driver(DriverConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {}
}
