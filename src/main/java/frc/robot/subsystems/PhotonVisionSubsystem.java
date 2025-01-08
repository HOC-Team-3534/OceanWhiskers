package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PhotonVisionSubsystem implements Subsystem {
    List<PhotonCamera> cameras = new ArrayList<>();

    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public PhotonVisionSubsystem(String... names) {
        for (String name : names) {
            cameras.add(new PhotonCamera(name));
        }
    }

    @Override
    public void periodic() {
        for (PhotonCamera camera : cameras) {
            var results = camera.getAllUnreadResults();
            for (var result : results) {
                result.getMultiTagResult().ifPresent(r -> {
                    Transform3d fieldToCamera = r.estimatedPose.best;
                });
            }
        }
    }
}
