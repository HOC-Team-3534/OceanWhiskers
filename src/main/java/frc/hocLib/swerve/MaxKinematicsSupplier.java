package frc.hocLib.swerve;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface MaxKinematicsSupplier {
    public LinearVelocity getMaxVelocity();

    public AngularVelocity getMaxAngularVelocity();
}
