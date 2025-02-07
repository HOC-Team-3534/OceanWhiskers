package frc.hocLib.talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.hocLib.util.CanDeviceId;

public final class TalonSRXFactory {
    public static TalonSRXConfiguration loadConfiguration(TalonSRX motor) {
        var talonSRXConfig = new TalonSRXConfiguration();
        motor.getAllConfigs(talonSRXConfig);

        return talonSRXConfig;
    }

    public static TalonSRX createPermanentFollower(CanDeviceId followerId, TalonSRX leader) {
        var follower = followerId.toTalonSRX();

        follower.configAllSettings(loadConfiguration(leader));

        follower.set(ControlMode.Follower, leader.getDeviceID());

        return follower;
    }
}
