package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.hocLib.HocSubsystem;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.hocLib.util.CachedValue;
import java.util.Optional;
import lombok.Setter;

public class VisionSystem extends HocSubsystem {
    public static class VisionConfig extends HocSubsystem.Config {
        @Setter private Distance frontXOffset = Inches.of(20.5 / 2);
        @Setter private Distance frontYOffset = Inches.of(20.0 / 2);
        @Setter private Distance rearXOffset = Inches.of(20.5 / 2);
        @Setter private Distance rearYOffset = Inches.of(20.5 / 2);

        @Setter private Angle fl_yaw = Degrees.of(-45);
        @Setter private Angle fr_yaw = Degrees.of(45);
        @Setter private Angle rl_yaw = Degrees.of(135);
        @Setter private Angle rr_yaw = Degrees.of(-135);

        @Setter private Distance cameraHeightOffGround = Inches.of(10.75);
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

    private final CachedValue<Optional<Distance>> cachedDistanceToAlignLeftPositive,
            cachedDistanceToAlignFwd;

    public VisionSystem(VisionConfig config) {
        super(config);
        this.config = config;

        fl_camera = new PhotonCameraPlus("fl_camera", config.getFLRobotToCamera());
        fr_camera = new PhotonCameraPlus("fr_camera", config.getFRRobotToCamera());
        rl_camera = new PhotonCameraPlus("rl_camera", config.getRLRobotToCamera());
        rr_camera = new PhotonCameraPlus("rr_camera", config.getRRRobotToCamera());

        cachedDistanceToAlignLeftPositive =
                new CachedValue<>(this::updateDistanceToAlignLeftPositive);

        cachedDistanceToAlignFwd = new CachedValue<>(this::updateDistanceToAlignFwd);
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

            // TODO: try latest photonvision release with new PNP solver from 6238
        }
    }

    public Optional<Distance> getDistanceToAlignLeftPositive() {
        return cachedDistanceToAlignLeftPositive.get();
    }

    private Optional<Distance> updateDistanceToAlignLeftPositive() {
        var fromLeft = fl_camera.getLatestLeftPostiveToTag();
        var fromRight = fr_camera.getLatestLeftPostiveToTag();

        if (fromLeft.isEmpty() && fromRight.isEmpty()) return Optional.empty();
        if (fromLeft.isEmpty()) return fromRight;
        if (fromRight.isEmpty()) return fromLeft;
        return Optional.of(fromLeft.get().plus(fromRight.get()).div(2));
    }

    public Optional<Distance> getDistanceToAlignFwd() {
        return cachedDistanceToAlignFwd.get();
    }

    private Optional<Distance> updateDistanceToAlignFwd() {
        var fromLeft = fl_camera.getLatestFwdToTag();
        var fromRight = fr_camera.getLatestFwdToTag();

        if (fromLeft.isEmpty() && fromRight.isEmpty()) return Optional.empty();
        if (fromLeft.isEmpty()) return fromRight;
        if (fromRight.isEmpty()) return fromLeft;
        return Optional.of(fromLeft.get().plus(fromRight.get()).div(2));
    }

    private static Transform3d calcRobotToCam(Translation2d xy, Distance height, Rotation3d rot) {
        return new Transform3d(new Translation3d(xy.getX(), xy.getY(), height.in(Meters)), rot);
    }

    private static Transform3d rotateAroundCenter(Transform3d vec, Rotation3d rot) {
        return rotateAroundPoint(vec, new Translation3d(), rot);
    }

    private static Transform3d rotateAroundPoint(
            Transform3d vec, Translation3d point, Rotation3d rot) {
        // 1. Translate the camera's position relative to the point of rotation.
        Translation3d translatedPosition = vec.getTranslation().minus(point);

        // 2. Apply the rotation to the translated position.
        Translation3d rotatedPosition = translatedPosition.rotateBy(rot);

        // 3. Translate the rotated position back by adding the point of rotation.
        rotatedPosition = rotatedPosition.plus(point);

        // 4. Apply the same rotation to the camera's rotation (orientation).
        Rotation3d rotatedRotation = rot.rotateBy(vec.getRotation());

        // 5. Return the transformed camera position and rotation.
        return new Transform3d(rotatedPosition, rotatedRotation);
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
