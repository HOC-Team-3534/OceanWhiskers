package frc.hocLib.motionmagic;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public final class MotionMagicV5 {
    public static double convV6toV5(
            double kvVoltsPerRotationPerSecond, Voltage supplyVoltage, double ticksPerRotation) {
        return kvVoltsPerRotationPerSecond
                * (1023 / supplyVoltage.in(Volts))
                * (10 / ticksPerRotation);
    }
}
