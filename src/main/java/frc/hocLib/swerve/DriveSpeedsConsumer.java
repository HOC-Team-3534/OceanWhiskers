package frc.hocLib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveSpeedsConsumer {
    public void driveWithSpeeds(ChassisSpeeds speeds);
}
