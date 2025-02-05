package frc.hocLib;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HocSubsystem extends Subsystem {
    default void setupStates() {}

    default void setupDefaultCommand() {}
}
