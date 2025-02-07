package frc.hocLib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public abstract class HocSubsystem extends SubsystemBase {
    @Getter private Config config;

    public HocSubsystem(Config config) {
        this.config = config;
        HocRobot.add(this);
    }

    public boolean isAttached() {
        return config.attached;
    }

    public abstract void setupBindings();

    public abstract void setupDefaultCommand();

    public static class Config {
        @Getter @Setter private String name;
        @Getter @Setter private boolean attached = true;

        public Config(String name) {
            this.name = name;
        }
    }
}
