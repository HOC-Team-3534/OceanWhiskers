package frc.hocLib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

public abstract class HocRobot extends TimedRobot {

    /** Create a single static instance of all of your subsystems */
    private static final ArrayList<HocSubsystem> subsystems = new ArrayList<>();

    public static void add(HocSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public HocRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    protected static void setupDefaultCommands() {
        // Setup Default Commands for all subsystems
        subsystems.forEach(HocSubsystem::setupDefaultCommand);
    }

    protected static void setupStates() {
        // Bind Triggers for all subsystems
        subsystems.forEach(HocSubsystem::setupBindings);
    }

    @Override
    public void disabledInit() {
        subsystems.forEach(HocSubsystem::disabledInit);
    }
}
