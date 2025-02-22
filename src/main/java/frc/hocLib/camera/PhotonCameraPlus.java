package frc.hocLib.camera;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;
import java.util.HashSet;
import java.util.Optional;
import lombok.Getter;
import lombok.experimental.Delegate;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraPlus {
    private static final Swerve swerve = Robot.getSwerve();

    final String subsystemName = "Vision";

    @Delegate final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;

    @Getter private Optional<Distance> latestLeftPostiveToTag = Optional.empty();
    @Getter private Optional<Distance> latestFwdToTag = Optional.empty();

    // The field from AprilTagFields will be different depending on the game.
    static AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    static HashSet<Integer> HIGH_TAGS = new HashSet<>();

    public PhotonCameraPlus(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        poseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        HIGH_TAGS.add(4);
        HIGH_TAGS.add(5);
        HIGH_TAGS.add(14);
        HIGH_TAGS.add(15);

        HIGH_TAGS.add(1);
        HIGH_TAGS.add(2);
        HIGH_TAGS.add(12);
        HIGH_TAGS.add(13);
    }

    public void update() {
        var results = camera.getAllUnreadResults();

        // TODO: validate isConnected is updating properly and catching issues that arise with
        // camera connection
        logConnected();

        for (var result : results) {
            var estimate = poseEstimator.update(result);

            logTrackedTarget(result.getBestTarget());

            estimate.ifPresent(
                    estmt -> {
                        var poseDifference =
                                estmt.estimatedPose
                                        .toPose2d()
                                        .relativeTo(swerve.getState().Pose)
                                        .getTranslation()
                                        .getNorm();

                        double sumTargetArea = 0.0;
                        boolean hasHighUpTags = false;
                        for (var tg : estmt.targetsUsed) {
                            sumTargetArea += tg.getArea();
                            if (HIGH_TAGS.contains(tg.fiducialId)) {
                                hasHighUpTags = true;
                                break;
                            }
                        }

                        var avgTargetArea = sumTargetArea / estmt.targetsUsed.size();

                        double xyStds = 2.0;
                        // TODO: consider modifying weights to help with center camera and alignment
                        if (estmt.targetsUsed.size() >= 2 && avgTargetArea > 0.1) xyStds = 0.2;
                        else if (hasHighUpTags && avgTargetArea > 0.2) xyStds = 0.5;
                        else if (avgTargetArea > 0.8 && poseDifference < 0.5) xyStds = 0.5;
                        else if (avgTargetArea > 0.1 && poseDifference < 0.3) xyStds = 1.0;
                        else if (estmt.targetsUsed.size() > 1) xyStds = 1.2;

                        var visionMeasurementStdDevs =
                                VecBuilder.fill(xyStds, xyStds, Degrees.of(50).in(Radians));

                        swerve.addVisionMeasurement(
                                estmt.estimatedPose.toPose2d(),
                                Utils.fpgaToCurrentTime(estmt.timestampSeconds),
                                visionMeasurementStdDevs);

                        var targetPose =
                                aprilTagFieldLayout.getTagPose(estmt.targetsUsed.get(0).fiducialId);

                        latestLeftPostiveToTag =
                                targetPose.map(
                                        tp ->
                                                estmt.estimatedPose
                                                        .toPose2d()
                                                        .minus(tp.toPose2d())
                                                        .getMeasureY());

                        latestFwdToTag =
                                targetPose.map(
                                        tp ->
                                                estmt.estimatedPose
                                                        .toPose2d()
                                                        .minus(tp.toPose2d())
                                                        .getMeasureX());
                    });
        }
    }

    void logConnected() {
        putBoolean("isConnected", isConnected());
    }

    void logTrackedTarget(PhotonTrackedTarget target) {
        if (target == null) return;

        var position = target.bestCameraToTarget.getTranslation();
        var rotation = target.bestCameraToTarget.getRotation();

        putNumber("Target ID", target.fiducialId);

        putNumber("To Target X", position.getMeasureX().in(Inches));
        putNumber("To Target Y", position.getMeasureY().in(Inches));
        putNumber("To Target Z", position.getMeasureZ().in(Inches));
        putNumber("To Target Roll", rotation.getMeasureX().in(Degrees));
        putNumber("To Target Pitch", rotation.getMeasureY().in(Degrees));
        putNumber("To Target Yaw", rotation.getMeasureZ().in(Degrees));
    }

    void putNumber(String key, double value) {
        SmartDashboard.putNumber(subsystemName + "/" + getName() + "/" + key, value);
    }

    void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(subsystemName + "/" + getName() + "/" + key, value);
    }
}
