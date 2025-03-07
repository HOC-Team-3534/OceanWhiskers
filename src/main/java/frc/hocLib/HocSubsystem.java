package frc.hocLib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
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

    @Getter
    @Setter
    @NonNull
    @RequiredArgsConstructor
    public static class Config {
        final String name;
        boolean attached = true;
        boolean testing = false;
    }
}
