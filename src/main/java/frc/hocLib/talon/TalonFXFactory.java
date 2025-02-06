package frc.hocLib.talon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.hocLib.util.CanDeviceId;

public final class TalonFXFactory {
    public static TalonFXConfiguration loadConfiguration(TalonFX motor) {
        var talonFXConfig = new TalonFXConfiguration();
        motor.getConfigurator().refresh(talonFXConfig);

        return talonFXConfig;
    }

    public static TalonFX createPermanentFollower(
            CanDeviceId followerId, TalonFX leader, boolean opposeLeader) {
        if (!followerId.getBus().equals(leader.getNetwork())) {
            throw new RuntimeException("Leader and Follower Talons must be on the same CANbus");
        }

        var follower = followerId.toTalonFX();

        follower.getConfigurator().apply(loadConfiguration(leader));

        follower.setControl(new Follower(leader.getDeviceID(), opposeLeader));

        return follower;
    }
}
