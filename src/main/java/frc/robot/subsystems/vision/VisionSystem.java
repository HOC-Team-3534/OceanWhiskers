package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.HocSubsystem;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.hocLib.camera.StdDevCategory;
import frc.reefscape.FieldAndTags2025;
import frc.robot.Robot;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.With;
import lombok.experimental.Delegate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem extends HocSubsystem {
    @Getter
    @Setter
    @With
    @NonNull
    @RequiredArgsConstructor
    @AllArgsConstructor
    public static class VisionConfig {
        @Delegate HocSubsystem.Config superConfig = new HocSubsystem.Config("Photon Vision");
        PhotonCameraPlus.Config<VisionStdDevCategory2025> baseConfig =
                PhotonCameraPlus.Config.defaults(VisionStdDevCategory2025.class)
                        .withAprilTagFieldLayout(FieldAndTags2025.APRIL_TAG_FIELD_LAYOUT);

        Distance frontX = Inches.of(20.5 / 2);
        Distance frontY = Inches.of(20.0 / 2);
        Distance rearX = Inches.of(20.5 / 2);
        Distance rearY = Inches.of(20.5 / 2);

        Angle fl_yaw = Degrees.of(-45);
        Angle fr_yaw = Degrees.of(45);
        Angle rl_yaw = Degrees.of(135);
        Angle rr_yaw = Degrees.of(-135);

        Distance cameraHeightOffGround = Inches.of(Robot.isReal() ? 10.75 : 13.0);
        Angle cameraPitch = Degrees.of(15.0);

        static Transform3d calcRobotToCam(Translation2d xy, Distance height, Rotation3d rot) {
            return new Transform3d(new Translation3d(xy.getX(), xy.getY(), height.in(Meters)), rot);
        }

        Transform3d calcRobotToCam(Distance xOffset, Distance yOffset, Angle yawOffset) {
            return calcRobotToCam(
                    new Translation2d(xOffset, yOffset),
                    cameraHeightOffGround,
                    new Rotation3d(Degrees.zero(), cameraPitch, yawOffset));
        }

        public Transform3d getRobotToCam(String cameraName) {
            return switch (cameraName) {
                case "fl_camera":
                    yield calcRobotToCam(frontX, frontY, fl_yaw);
                case "fr_camera":
                    yield calcRobotToCam(frontX, frontY.unaryMinus(), fr_yaw);
                case "rl_camera":
                    yield calcRobotToCam(rearX.unaryMinus(), rearY, rl_yaw);
                case "rr_camera":
                    yield calcRobotToCam(rearX.unaryMinus(), rearY.unaryMinus(), rr_yaw);
                default:
                    throw new IllegalArgumentException(
                            "No valid camera matching the name: " + cameraName);
            };
        }

        public PhotonCameraPlus.Config<VisionStdDevCategory2025> getCameraConfig(
                String cameraName) {
            return baseConfig.withCameraSpecifics(cameraName, getRobotToCam(cameraName));
        }
    }

    final PhotonCameraPlus<VisionStdDevCategory2025> fl_camera, fr_camera, rl_camera, rr_camera;

    @SuppressWarnings("unused")
    private VisionConfig config;

    private VisionSystemSim visionSim;

    public VisionSystem(VisionConfig config) {
        super(config.getSuperConfig());
        this.config = config;

        fl_camera = PhotonCameraPlus.create(config.getCameraConfig("fl_camera"));
        fr_camera = PhotonCameraPlus.create(config.getCameraConfig("fr_camera"));
        rl_camera = PhotonCameraPlus.create(config.getCameraConfig("rl_camera"));
        rr_camera = PhotonCameraPlus.create(config.getCameraConfig("rr_camera"));

        if (Robot.isSimulation()) {

            visionSim = new VisionSystemSim("main");

            visionSim.addAprilTags(config.baseConfig.getAprilTagFieldLayout());

            fl_camera.addToVisionSim(visionSim);
            fr_camera.addToVisionSim(visionSim);
            rl_camera.addToVisionSim(visionSim);
            rr_camera.addToVisionSim(visionSim);
        }
    }

    @Override
    public void periodic() {
        if (isAttached()) {
            // See
            // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java#L382C7-L401C14 for conditional stddev example from 2024 from frc team 254 cheezy poofs
            var robot = Robot.getSwerve().getPose();

            fl_camera.update(robot, this::addVisionMeasurement);
            fr_camera.update(robot, this::addVisionMeasurement);
            rl_camera.update(robot, this::addVisionMeasurement);
            rr_camera.update(robot, this::addVisionMeasurement);

            poseEstimatorMap.forEach(
                    (tag, estimator) -> {
                        estimator
                                .getFirst()
                                .update(
                                        estimator.getFirst().getEstimatedPosition().getRotation(),
                                        new SwerveModulePosition[] {
                                            new SwerveModulePosition(), new SwerveModulePosition()
                                        });
                    });
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(Robot.getSwerve().getState().Pose);
    }

    private Map<Integer, Pair<SwerveDrivePoseEstimator, Timer>> poseEstimatorMap = new HashMap<>();

    public void addVisionMeasurement(
            EstimatedRobotPose estimate, VisionStdDevCategory2025 category, List<Integer> tagIds) {
        var poseEstimate = estimate.estimatedPose.toPose2d();

        Robot.getSwerve()
                .addVisionMeasurement(
                        poseEstimate,
                        Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                        category.getStdDevs());

        for (var tag : tagIds) {
            var estimator = poseEstimatorMap.get(tag);

            if (estimator == null) {
                estimator =
                        new Pair<SwerveDrivePoseEstimator, Timer>(
                                new SwerveDrivePoseEstimator(
                                        new SwerveDriveKinematics(
                                                new Translation2d(), new Translation2d()),
                                        poseEstimate.getRotation(),
                                        new SwerveModulePosition[] {
                                            new SwerveModulePosition(), new SwerveModulePosition()
                                        },
                                        poseEstimate,
                                        VecBuilder.fill(1000.0, 1000.0, 1000.0),
                                        VecBuilder.fill(0.9, 0.9, 0.9)),
                                new Timer());
                poseEstimatorMap.put(tag, estimator);
            }

            estimator
                    .getFirst()
                    .addVisionMeasurement(
                            poseEstimate, estimate.timestampSeconds, category.getStdDevs());
            estimator.getSecond().restart();
        }
    }

    public Optional<Pose2d> getPoseEstimateByTag(int tagId) {
        return Optional.ofNullable(poseEstimatorMap.get(tagId))
                .flatMap(
                        estimator ->
                                estimator.getSecond().hasElapsed(0.5)
                                        ? Optional.empty()
                                        : Optional.of(estimator.getFirst().getEstimatedPosition()));
    }

    static HashSet<Integer> HIGH_TAGS = new HashSet<>();

    static {
        HIGH_TAGS.add(4);
        HIGH_TAGS.add(5);
        HIGH_TAGS.add(14);
        HIGH_TAGS.add(15);

        HIGH_TAGS.add(1);
        HIGH_TAGS.add(2);
        HIGH_TAGS.add(12);
        HIGH_TAGS.add(13);
    }

    private static boolean hasHighUpTags(EstimatedRobotPose estimate) {
        for (var tg : estimate.targetsUsed) if (HIGH_TAGS.contains(tg.fiducialId)) return true;
        return false;
    }

    @RequiredArgsConstructor
    enum VisionStdDevCategory2025 implements StdDevCategory<VisionStdDevCategory2025> {
        MultipleTags(0.2, 0.1),
        HighUpTag(0.5, 0.2),
        RelativelyClose(0.5, 0.8),
        RelativelyFarButSmallDifference(1.0, 0.1),
        MultipleTagsButFar(1.2, 0.0),
        Default(2.0, 0.0);
        final double xyStdDev, minAverageTargetArea;

        @Override
        public boolean fitsCriteria(
                EstimatedRobotPose estimate, double avgTargetArea, double poseDifference) {
            var meetsMultiTagIfNeeded =
                    switch (this) {
                        case MultipleTags, MultipleTagsButFar:
                            yield estimate.targetsUsed.size() > 1;
                        default:
                            yield true;
                    };

            var meetsHighUpTagIfNeeded =
                    switch (this) {
                        case HighUpTag:
                            yield hasHighUpTags(estimate);
                        default:
                            yield true;
                    };

            var maxPoseDifference =
                    switch (this) {
                        case RelativelyClose:
                            yield 0.5;
                        case RelativelyFarButSmallDifference:
                            yield 0.3;
                        default:
                            yield Double.POSITIVE_INFINITY;
                    };

            return meetsMultiTagIfNeeded
                    && meetsHighUpTagIfNeeded
                    && avgTargetArea >= minAverageTargetArea
                    && poseDifference <= maxPoseDifference;
        }

        @Override
        public Vector<N3> getStdDevs() {
            return VecBuilder.fill(xyStdDev, xyStdDev, Degrees.of(50).in(Radians));
        }
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
