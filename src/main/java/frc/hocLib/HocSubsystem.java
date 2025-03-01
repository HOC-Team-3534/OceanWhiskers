package frc.hocLib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;

public abstract class HocSubsystem extends SubsystemBase {
    @Getter private Config config;

    public HocSubsystem(Config config) {
        this.config = config;
        HocRobot.add(this);
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    public Trigger isTesting = new Trigger(() -> config.isTesting());

    public abstract void setupBindings();

    public abstract void setupDefaultCommand();

    public void disabledInit() {}

    public Trigger isCurrentCommand(String name) {
        return new Trigger(() -> this.getCurrentCommand().getName().equals(name));
    }

    public static class Config {
        @Getter @Setter private String name;
        @Getter @Setter private boolean attached = true;

        @Getter private boolean testing;

        public Config(String name) {
            this.name = name;
        }

        public Config testing() {
            testing = true;
            return this;
        }
    }
}
