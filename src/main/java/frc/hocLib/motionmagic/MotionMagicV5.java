package frc.hocLib.motionmagic;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public final class MotionMagicV5 {
    public static double convV6toV5(
            double kvVoltsPerRotationPerSecond, TalonSRX srx, double ticksPerRotation) {
        return kvVoltsPerRotationPerSecond * (1023 / srx.getBusVoltage()) * (10 / ticksPerRotation);
    }
}
