package frc.robot.utils.camera;

import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionCameraConfig {
    public final String name;
    public final Transform3d robotToCam;

    public PhotonVisionCameraConfig(String name, Transform3d robotToCam) {
        this.name = name;
        this.robotToCam = robotToCam;
    }
}
