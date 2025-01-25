package frc.robot.utils.camera;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashSet;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotContainer;

public class PhotonCameraPlus {
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;

    // The field from AprilTagFields will be different depending on the game.
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    static HashSet<Integer> HIGH_TAGS = new HashSet<>();

    public PhotonCameraPlus(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        HIGH_TAGS.add(4);
        HIGH_TAGS.add(5);
        HIGH_TAGS.add(14);
        HIGH_TAGS.add(15);
    }

    public void update() {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
            var estimate = poseEstimator.update(result);

            estimate.ifPresent(estmt -> {

                var poseDifference = estmt.estimatedPose.toPose2d()
                        .relativeTo(RobotContainer.getSwerveDriveSubsystem().getState().Pose)
                        .getTranslation().getNorm();

                double sumTargetArea = 0.0;
                boolean hasHighUpTags = false;
                for (var tg : estmt.targetsUsed) {
                    sumTargetArea += tg.getArea();
                    if (HIGH_TAGS.contains(tg.fiducialId))
                        hasHighUpTags = true;
                }

                var avgTargetArea = sumTargetArea / estmt.targetsUsed.size();

                double xyStds = 2.0;
                if (estmt.targetsUsed.size() >= 2 && avgTargetArea > 0.1)
                    xyStds = 0.2;
                else if (hasHighUpTags && avgTargetArea > 0.2)
                    xyStds = 0.5;
                else if (avgTargetArea > 0.8 && poseDifference < 0.5)
                    xyStds = 0.5;
                else if (avgTargetArea > 0.1 && poseDifference < 0.3)
                    xyStds = 1.0;
                else if (estmt.targetsUsed.size() > 1)
                    xyStds = 1.2;

                var visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Degrees.of(50).in(Radians));

                RobotContainer.getSwerveDriveSubsystem().addVisionMeasurement(estmt.estimatedPose.toPose2d(),
                        Utils.fpgaToCurrentTime(estmt.timestampSeconds), visionMeasurementStdDevs);
            });
        }
    }
}
