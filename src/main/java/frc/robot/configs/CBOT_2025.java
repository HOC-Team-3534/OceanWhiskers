package frc.robot.configs;

import frc.robot.Robot.Config;

public class CBOT_2025 extends Config {

    public CBOT_2025() {
        this.algaeWheel.setAttached(false);
        this.elevator.setAttached(false);
        this.jaws.setAttached(false);
        this.lights.setAttached(false);
        this.tusks.setAttached(false);

        this.vision.setAttached(false);
        this.swerve.setAttached(true);

        this.driver.setAttached(true);
        this.codriver.setAttached(true);
    }
}
