package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotContainer;

public class PhotonCameraPlus {
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;

    // The field from AprilTagFields will be different depending on the game.
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public PhotonCameraPlus(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void update() {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
            var estimate = poseEstimator.update(result);

            estimate.ifPresent(estmt -> {
                var ambiguityAcceptable = estmt.targetsUsed.get(0).getPoseAmbiguity() < 0.2;

                // TODO: Might also need to check the estimate pose for the robot is not off the flat ground

                if (ambiguityAcceptable) {
                    // TODO: Does added vision measurement need to have a matrix of custom standard deviations? 
                    RobotContainer.getSwerveDriveSubsystem().addVisionMeasurement(estmt.estimatedPose.toPose2d(),
                            estmt.timestampSeconds);
                }
            });
        }
    }
}
