package frc.hocLib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TuningCommand extends InstantCommand {
    public static boolean TUNING_MODE;

    public TuningCommand(String key, Runnable runnable) {
        super(runnable);
        if (TUNING_MODE) {
            SmartDashboard.putData("Tuning/" + key, this);
        }
    }

    public static TuningCommand create(String key, Runnable runnable) {
        return new TuningCommand(key, runnable);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
