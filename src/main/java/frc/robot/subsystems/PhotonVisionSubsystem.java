package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PhotonCameraPlus;

public class PhotonVisionSubsystem implements Subsystem {
    PhotonCameraPlus fl_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus fr_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus rl_camera = new PhotonCameraPlus("fl_camera", new Transform3d());
    PhotonCameraPlus rr_camera = new PhotonCameraPlus("fl_camera", new Transform3d());

    @Override
    public void periodic() {
        fl_camera.update();
        fr_camera.update();
        rl_camera.update();
        rr_camera.update();
    }
}
