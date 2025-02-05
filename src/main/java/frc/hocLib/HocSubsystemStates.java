package frc.hocLib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface HocSubsystemStates {
    default Command getDefaultCommand() {
        return Commands.none();
    }

    default void setupStates() {}
}
