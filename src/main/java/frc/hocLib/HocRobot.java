package frc.hocLib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

public class HocRobot extends TimedRobot {

    /** Create a single static instance of all of your subsystems */
    private static final ArrayList<HocSubsystem> subsystems = new ArrayList<>();

    public static void add(HocSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public HocRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    protected void setupDefaultCommands() {
        // Setup Default Commands for all subsystems
        subsystems.forEach(HocSubsystem::setupDefaultCommand);
    }
}
