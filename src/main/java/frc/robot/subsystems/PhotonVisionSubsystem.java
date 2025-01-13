package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.camera.PhotonCameraPlus;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCameraPlus fl_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus fr_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus rl_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus rr_camera = new PhotonCameraPlus("fl_camera", new Transform3d());

    @Override
    public void periodic() {
        // See https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java#L382C7-L401C14 for conditional stddev example from 2024 from frc team 254 cheezy poofs
        fl_camera.update();
        fr_camera.update();
        rl_camera.update();
        rr_camera.update();
    }
}
