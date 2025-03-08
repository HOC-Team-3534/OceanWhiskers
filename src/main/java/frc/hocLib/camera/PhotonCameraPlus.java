package frc.hocLib.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.Logging;
import frc.hocLib.util.GeomUtil;
import frc.hocLib.util.LoggedTunableNumber;
import frc.hocLib.util.TuningCommand;
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
    private static final LoggedTunableNumber calibrationMinTagArea =
            new LoggedTunableNumber("PhotonCameraPlus/MinTagArea");

    private static final LoggedTunableNumber calibrationKnownX =
            new LoggedTunableNumber("PhotonCameraPlus/Known/X");
    private static final LoggedTunableNumber calibrationKnownY =
            new LoggedTunableNumber("PhotonCameraPlus/Known/Y");
    private static final LoggedTunableNumber calibrationKnownOmegaDegrees =
            new LoggedTunableNumber("PhotonCameraPlus/Known/OmegaDegrees");

    private static final LoggedTunableNumber calibrationTagId =
            new LoggedTunableNumber("PhotonCameraPlus/TagId");

    private final List<Transform3d> calibrationRobotToCameras = new ArrayList<>();

    private static final List<PhotonCameraPlus<?>> allInstances = new ArrayList<>();

    static {
        TuningCommand.create(
                "PhotonCameraPlus/Calibrate",
                () -> allInstances.forEach(PhotonCameraPlus::calibrateWithLatestResult));
        TuningCommand.create(
                "PhotonCameraPlus/Clear Calibrations",
                () -> allInstances.forEach(PhotonCameraPlus::clearCalibration));
        TuningCommand.create(
                "PhotonCameraPlus/Print Calibrations",
                () -> allInstances.forEach(PhotonCameraPlus::printCalibration));

        calibrationMinTagArea.initDefault(0.2);

        calibrationKnownX.initDefault(0.0);
        calibrationKnownY.initDefault(0.0);
        calibrationKnownOmegaDegrees.initDefault(0.0);

        calibrationTagId.initDefault(1.0);
    }

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

    @Getter private PhotonPipelineResult latestResultWithTargets;

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

        allInstances.add(this);
    }

    public void addToVisionSim(VisionSystemSim visionSim) {
        if (RobotBase.isSimulation()) {
            visionSim.addCamera(cameraSim, config.robotToCamera);
        }
    }

    public void update(Pose2d robot, VisionEstimateConsumer<T> visionOutput) {

        Logging.log(camera.getName(), this);

        if (isConnected()) {

            var results = camera.getAllUnreadResults();

            for (var result : results) {
                processResult(result, robot, visionOutput);
            }
        }
        if (calibrationKnownX.hasChanged(100)
                || calibrationKnownY.hasChanged(100)
                || calibrationKnownOmegaDegrees.hasChanged(100))
            Logging.log(
                    "Tuning/PhotonCameraPlus/FieldToRobot",
                    new Pose3d(
                            new Pose2d(
                                    calibrationKnownX.get(),
                                    calibrationKnownY.get(),
                                    Rotation2d.fromDegrees(calibrationKnownOmegaDegrees.get()))));

        if (calibrationTagId.hasChanged(100))
            Logging.log(
                    "Tuning/PhotonCameraPlus/TagPose",
                    config.aprilTagFieldLayout.getTagPose((int) calibrationTagId.get()).get());
    }

    private void processResult(
            PhotonPipelineResult result, Pose2d robot, VisionEstimateConsumer<T> visionOutput) {
        var estimate = poseEstimator.update(result);

        estimate.ifPresent(
                estmt -> {
                    latestResultWithTargets = result;

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
                });
    }

    private static double calcAvgTargetArea(EstimatedRobotPose estmt) {
        double sumTargetArea = 0.0;
        for (var tg : estmt.targetsUsed) {
            sumTargetArea += tg.getArea();
        }
        return sumTargetArea / estmt.targetsUsed.size();
    }

    private void calibrateWithLatestResult() {
        if (latestResultWithTargets == null
                || Timer.getFPGATimestamp() - latestResultWithTargets.getTimestampSeconds() > 0.250)
            return;

        PhotonTrackedTarget bestTarget = latestResultWithTargets.getBestTarget();

        if (calibrationMinTagArea.get() < 0.2 || bestTarget.poseAmbiguity > 0.2) return;

        Pose3d fieldToRobot =
                new Pose3d(
                        new Pose2d(
                                calibrationKnownX.get(),
                                calibrationKnownY.get(),
                                Rotation2d.fromDegrees(calibrationKnownOmegaDegrees.get())));

        Pose3d fieldToTag = config.aprilTagFieldLayout.getTagPose(bestTarget.fiducialId).get();

        Transform3d cameraToTag = bestTarget.bestCameraToTarget;

        Transform3d tagToCamera = cameraToTag.inverse();

        Pose3d fieldToCamera = fieldToTag.transformBy(tagToCamera);

        Transform3d robotToCamera = new Transform3d(fieldToRobot, fieldToCamera);

        calibrationRobotToCameras.add(robotToCamera);

        var average = GeomUtil.averageTransform(calibrationRobotToCameras);

        Logging.log(camera.getName() + "/Calibration Robot To Camera", average);

        Logging.log(camera.getName() + "/Calibration Roll", average.getRotation().getX());
        Logging.log(camera.getName() + "/Calibration Pitch", average.getRotation().getY());
        Logging.log(camera.getName() + "/Calibration Yaw", average.getRotation().getZ());
    }

    private void clearCalibration() {
        calibrationRobotToCameras.clear();
    }

    private void printCalibration() {
        var average = GeomUtil.averageTransform(calibrationRobotToCameras);

        var rotation =
                new Rotation3d(average.getRotation().getQuaternion()) {
                    @Override
                    public String toString() {
                        return String.format("Rotation3d(%s, %s, %s)", getX(), getY(), getZ());
                    }
                };

        System.out.println();
        System.out.println(camera.getName() + ":");
        System.out.println(average.getTranslation());
        System.out.println(rotation);
        System.out.println();
    }
}
