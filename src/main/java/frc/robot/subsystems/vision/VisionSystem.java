package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.HocSubsystem;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.hocLib.swerve.VisionMeasurementWithIdsAdder;
import frc.reefscape.FieldAndTags2025;
import frc.robot.Robot;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import lombok.Setter;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem extends HocSubsystem implements VisionMeasurementWithIdsAdder {
    public static class VisionConfig extends HocSubsystem.Config {
        @Setter private Distance frontXOffset = Inches.of((20.5 / 2));
        @Setter private Distance frontYOffset = Inches.of(20.0 / 2);
        @Setter private Distance rearXOffset = Inches.of(20.5 / 2);
        @Setter private Distance rearYOffset = Inches.of(20.5 / 2);

        @Setter private Angle fl_yaw = Degrees.of(-45);
        @Setter private Angle fr_yaw = Degrees.of(45);
        @Setter private Angle rl_yaw = Degrees.of(135);
        @Setter private Angle rr_yaw = Degrees.of(-135);

        @Setter private Distance cameraHeightOffGround = Inches.of(Robot.isReal() ? 10.75 : 13.0);
        @Setter private Angle cameraPitch = Degrees.of(15.0);

        public VisionConfig() {
            super("Photon Vision");
            setAttached(false);
        }

        Transform3d calculateCameraOffset(Distance xOffset, Distance yOffset, Angle yawOffset) {
            return calcRobotToCam(
                    new Translation2d(xOffset, yOffset),
                    cameraHeightOffGround,
                    new Rotation3d(Degrees.zero(), cameraPitch, yawOffset));
        }

        public Transform3d getFLRobotToCamera() {
            return calculateCameraOffset(frontXOffset, frontYOffset, fl_yaw);
        }

        public Transform3d getFRRobotToCamera() {
            return calculateCameraOffset(frontXOffset, frontYOffset.unaryMinus(), fr_yaw);
        }

        public Transform3d getRLRobotToCamera() {
            return calculateCameraOffset(rearXOffset.unaryMinus(), rearYOffset, rl_yaw);
        }

        public Transform3d getRRRobotToCamera() {
            return calculateCameraOffset(
                    rearXOffset.unaryMinus(), rearYOffset.unaryMinus(), rr_yaw);
        }
    }

    final PhotonCameraPlus fl_camera, fr_camera, rl_camera, rr_camera;

    @SuppressWarnings("unused")
    private VisionConfig config;

    private VisionSystemSim visionSim;

    public VisionSystem(VisionConfig config) {
        super(config);
        this.config = config;

        var swerve = Robot.getSwerve();

        fl_camera = new PhotonCameraPlus("fl_camera", config.getFLRobotToCamera(), this, swerve);
        fr_camera = new PhotonCameraPlus("fr_camera", config.getFRRobotToCamera(), this, swerve);
        rl_camera = new PhotonCameraPlus("rl_camera", config.getRLRobotToCamera(), this, swerve);
        rr_camera = new PhotonCameraPlus("rr_camera", config.getRRRobotToCamera(), this, swerve);

        if (Robot.isSimulation()) {

            visionSim = new VisionSystemSim("main");

            visionSim.addAprilTags(FieldAndTags2025.APRIL_TAG_FIELD_LAYOUT);

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
            fl_camera.update();
            fr_camera.update();
            rl_camera.update();
            rr_camera.update();

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

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs,
            List<Integer> tagIds) {
        Robot.getSwerve()
                .addVisionMeasurement(
                        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

        for (var tag : tagIds) {
            var estimator = poseEstimatorMap.get(tag);

            if (estimator == null) {
                estimator =
                        new Pair<SwerveDrivePoseEstimator, Timer>(
                                new SwerveDrivePoseEstimator(
                                        new SwerveDriveKinematics(
                                                new Translation2d(), new Translation2d()),
                                        visionRobotPoseMeters.getRotation(),
                                        new SwerveModulePosition[] {
                                            new SwerveModulePosition(), new SwerveModulePosition()
                                        },
                                        visionRobotPoseMeters,
                                        VecBuilder.fill(1000.0, 1000.0, 1000.0),
                                        VecBuilder.fill(0.9, 0.9, 0.9)),
                                new Timer());
                poseEstimatorMap.put(tag, estimator);
            }

            estimator
                    .getFirst()
                    .addVisionMeasurement(
                            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
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

    private static Transform3d calcRobotToCam(Translation2d xy, Distance height, Rotation3d rot) {
        return new Transform3d(new Translation3d(xy.getX(), xy.getY(), height.in(Meters)), rot);
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
