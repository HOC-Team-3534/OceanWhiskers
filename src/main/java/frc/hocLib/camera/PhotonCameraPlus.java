package frc.hocLib.camera;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.Logging;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import lombok.experimental.Delegate;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraPlus {

    final String subsystemName = "Vision";

    @Delegate final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;

    // The field from AprilTagFields will be different depending on the game.
    static AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    final Transform3d robotToCamera;

    static HashSet<Integer> HIGH_TAGS = new HashSet<>();

    private SimCameraProperties cameraProp = new SimCameraProperties();
    private PhotonCameraSim cameraSim;

    @Getter private List<PhotonTrackedTarget> latestEstimateTargets = new ArrayList<>();
    private Timer latestEstimateTimer = new Timer();

    public PhotonCameraPlus(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);

        if (Robot.isSimulation()) {

            cameraProp.setFPS(90);
            cameraProp.setCalibration(1920, 1200, Rotation2d.fromDegrees(104));

            cameraSim = new PhotonCameraSim(camera, cameraProp);
        }

        this.robotToCamera = robotToCamera;

        poseEstimator = createPoseEstimator();
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

    public void addToVisionSim(VisionSystemSim visionSim) {
        if (Robot.isSimulation()) {
            visionSim.addCamera(cameraSim, robotToCamera);
        }
        latestEstimateTimer.restart();
    }

    private PoseStrategy calculateCurrentPoseStrategy() {
        // return DriverStation.isFMSAttached()
        //         ? PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        //         : PoseStrategy.LOWEST_AMBIGUITY;

        return PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    }

    private PhotonPoseEstimator createPoseEstimator() {
        return new PhotonPoseEstimator(
                aprilTagFieldLayout, calculateCurrentPoseStrategy(), robotToCamera);
    }

    public void update() {
        if (!poseEstimator.getPrimaryStrategy().equals(calculateCurrentPoseStrategy())) {
            poseEstimator.setPrimaryStrategy(calculateCurrentPoseStrategy());
        }

        Logging.log(camera.getName(), this);

        if (isConnected()) {

            var results = camera.getAllUnreadResults();

            for (var result : results) {
                var estimate = poseEstimator.update(result);

                estimate.ifPresent(
                        estmt -> {
                            var poseDifference =
                                    estmt.estimatedPose
                                            .toPose2d()
                                            .relativeTo(Robot.getSwerve().getState().Pose)
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
                            // TODO: consider modifying weights to help with center camera and
                            // alignment
                            if (estmt.targetsUsed.size() >= 2 && avgTargetArea > 0.1) xyStds = 0.2;
                            else if (hasHighUpTags && avgTargetArea > 0.2) xyStds = 0.5;
                            else if (avgTargetArea > 0.8 && poseDifference < 0.5) xyStds = 0.5;
                            else if (avgTargetArea > 0.1 && poseDifference < 0.3) xyStds = 1.0;
                            else if (estmt.targetsUsed.size() > 1) xyStds = 1.2;

                            var visionMeasurementStdDevs =
                                    VecBuilder.fill(xyStds, xyStds, Degrees.of(50).in(Radians));

                            Robot.getSwerve()
                                    .addVisionMeasurement(
                                            estmt.estimatedPose.toPose2d(),
                                            Utils.fpgaToCurrentTime(estmt.timestampSeconds),
                                            visionMeasurementStdDevs);

                            latestEstimateTargets.clear();
                            latestEstimateTargets.addAll(estmt.targetsUsed);

                            latestEstimateTimer.restart();
                        });
            }
        }
    }

    public Optional<Transform2d> getRobotToTarget(
            int targetOfInterestId, Time timeLimitSinceLastDetected) {

        if (latestEstimateTimer.hasElapsed(timeLimitSinceLastDetected.in(Seconds)))
            return Optional.empty();

        var targetOfInterest =
                latestEstimateTargets.stream()
                        .filter((t) -> t.fiducialId == targetOfInterestId)
                        .findFirst();

        if (targetOfInterest.isEmpty()) return Optional.empty();

        var robotToTarget =
                new Pose3d()
                        .transformBy(robotToCamera)
                        .transformBy(targetOfInterest.get().bestCameraToTarget)
                        .transformBy(
                                new Transform3d(
                                        new Translation3d(), new Rotation3d(Rotation2d.k180deg)));

        return Optional.of(robotToTarget.toPose2d().minus(new Pose2d()));
    }
}
