package frc.hocLib.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.Logging;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.With;
import lombok.experimental.Delegate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraPlus<T extends Enum<T> & StdDevCategory<T>> {

    @Getter
    @With
    @RequiredArgsConstructor
    public static class Config<T> {
        final String name;
        final Transform3d robotToCamera;

        final double maxCloseTagArea;
        final AprilTagFieldLayout aprilTagFieldLayout;
        final PoseStrategy poseStrategy;
        final PoseStrategy multiTagFallbackPoseStrategy;
        final Class<T> categoryClass;

        private Config(String name, Transform3d robotToCamera, Config<T> baseConfig) {
            this(
                    name,
                    robotToCamera,
                    baseConfig.maxCloseTagArea,
                    baseConfig.aprilTagFieldLayout,
                    baseConfig.poseStrategy,
                    baseConfig.multiTagFallbackPoseStrategy,
                    baseConfig.categoryClass);
        }

        public static <T extends Enum<T> & StdDevCategory<T>> Config<T> defaults(
                Class<T> categoryClass) {
            return new Config<>(
                    null,
                    null,
                    0.1,
                    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseStrategy.LOWEST_AMBIGUITY,
                    categoryClass);
        }

        public Config<T> withCameraSpecifics(String name, Transform3d robotToCamera) {
            return new Config<>(name, robotToCamera, this);
        }

        public SimCameraProperties getSimCameraProperties() {
            var cameraProp = new SimCameraProperties();
            cameraProp.setFPS(90);
            cameraProp.setCalibration(1920, 1200, Rotation2d.fromDegrees(104));
            return cameraProp;
        }
    }

    private final Config<T> config;

    @Delegate final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;
    private PhotonCameraSim cameraSim;

    @Getter private List<PhotonTrackedTarget> latestEstimateTargets = new ArrayList<>();
    private Timer latestEstimateTimer = new Timer();

    public static <TT extends Enum<TT> & StdDevCategory<TT>> PhotonCameraPlus<TT> create(
            Config<TT> config) {
        return new PhotonCameraPlus<>(config);
    }

    public PhotonCameraPlus(Config<T> config) {
        this.config = config;

        camera = new PhotonCamera(config.name);

        if (RobotBase.isSimulation()) {
            cameraSim = new PhotonCameraSim(camera, config.getSimCameraProperties());
        }

        poseEstimator =
                new PhotonPoseEstimator(
                        config.aprilTagFieldLayout, config.poseStrategy, config.robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(config.multiTagFallbackPoseStrategy);
    }

    public void addToVisionSim(VisionSystemSim visionSim) {
        if (RobotBase.isSimulation()) {
            visionSim.addCamera(cameraSim, config.robotToCamera);
        }
        latestEstimateTimer.restart();
    }

    public void update(Pose2d robot, VisionEstimateConsumer<T> visionOutput) {

        Logging.log(camera.getName(), this);

        if (isConnected()) {

            var results = camera.getAllUnreadResults();

            for (var result : results) {
                processResult(result, robot, visionOutput);
            }
        }
    }

    private void processResult(
            PhotonPipelineResult result, Pose2d robot, VisionEstimateConsumer<T> visionOutput) {
        var estimate = poseEstimator.update(result);

        estimate.ifPresent(
                estmt -> {
                    // var visionMeasurementStdDevs =
                    //         calcStandardDevs(estmt, robot);
                    var avgTargetArea = calcAvgTargetArea(estmt);

                    var poseDifference =
                            estmt.estimatedPose
                                    .toPose2d()
                                    .relativeTo(robot)
                                    .getTranslation()
                                    .getNorm();

                    var stdDevCategory =
                            StdDevCategoryUtil.selectCategory(
                                    config.categoryClass, estmt, avgTargetArea, poseDifference);

                    ArrayList<Integer> closerTagIds = new ArrayList<>();

                    for (var tag : estmt.targetsUsed) {
                        if (tag.area >= avgTargetArea && tag.area > 0.1)
                            closerTagIds.add(tag.getFiducialId());
                    }

                    visionOutput.accept(estmt, stdDevCategory, closerTagIds);

                    Logging.log(camera.getName(), estmt, stdDevCategory);

                    latestEstimateTargets.clear();
                    latestEstimateTargets.addAll(estmt.targetsUsed);

                    latestEstimateTimer.restart();
                });
    }

    private static double calcAvgTargetArea(EstimatedRobotPose estmt) {
        double sumTargetArea = 0.0;
        for (var tg : estmt.targetsUsed) {
            sumTargetArea += tg.getArea();
        }
        return sumTargetArea / estmt.targetsUsed.size();
    }
}
