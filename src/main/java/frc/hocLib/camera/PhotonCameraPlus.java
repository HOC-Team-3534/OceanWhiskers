package frc.hocLib.camera;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.Logging;
import frc.hocLib.swerve.RobotPoseSupplier;
import frc.hocLib.swerve.VisionMeasurementAdder;
import frc.hocLib.swerve.VisionMeasurementWithIdsAdder;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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

    private final VisionMeasurementWithIdsAdder visionMeasurementAdder;
    private final RobotPoseSupplier robotPoseSupplier;

    public PhotonCameraPlus(
            String name,
            Transform3d robotToCamera,
            VisionMeasurementAdder visionMeasurementAdder,
            RobotPoseSupplier robotPoseSupplier) {
        this(
                name,
                robotToCamera,
                (visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs, tagIds) ->
                        visionMeasurementAdder.addVisionMeasurement(
                                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs),
                robotPoseSupplier);
    }

    public PhotonCameraPlus(
            String name,
            Transform3d robotToCamera,
            VisionMeasurementWithIdsAdder visionMeasurementAdder,
            RobotPoseSupplier robotPoseSupplier) {
        camera = new PhotonCamera(name);

        this.visionMeasurementAdder = visionMeasurementAdder;
        this.robotPoseSupplier = robotPoseSupplier;

        if (RobotBase.isSimulation()) {

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
        if (RobotBase.isSimulation()) {
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
                                            .relativeTo(robotPoseSupplier.getPose())
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

                            ArrayList<Integer> closerTagIds = new ArrayList<>();

                            for (var tag : estmt.targetsUsed) {
                                if (tag.area >= avgTargetArea && tag.area > 0.1)
                                    closerTagIds.add(tag.getFiducialId());
                            }

                            visionMeasurementAdder.addVisionMeasurement(
                                    estmt.estimatedPose.toPose2d(),
                                    Utils.fpgaToCurrentTime(estmt.timestampSeconds),
                                    visionMeasurementStdDevs,
                                    closerTagIds);

                            latestEstimateTargets.clear();
                            latestEstimateTargets.addAll(estmt.targetsUsed);

                            latestEstimateTimer.restart();
                        });
            }
        }
    }
}
