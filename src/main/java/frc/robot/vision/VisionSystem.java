package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.hocLib.HocSubsystem;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.robot.swerve.SwerveConfig;
import java.util.Optional;
import lombok.Getter;

public class VisionSystem extends HocSubsystem {
    public static class VisionConfig extends HocSubsystem.Config {
        @Getter
        private boolean centerCameraAttached = false;

        public VisionConfig() {
            super("Photon Vision");
            setAttached(false);
        }

        public VisionConfig configCenterCameraAttached(boolean attached) {
            centerCameraAttached = attached;
            return this;
        }
    }

    private SwerveConfig defaultSwerve = new SwerveConfig();

    private final Distance CAMERA_INSET_FROM_CANCODER = Inches.of(0.5);
    private final Distance CAMERA_HEIGHT_OFF_GROUND = Inches.of(10.75);
    private final Angle CAMERA_PITCH = Degrees.of(15.0);

    private final Translation2d cameraOffset = new Translation2d(CAMERA_INSET_FROM_CANCODER.in(Meters),
            Rotation2d.fromDegrees(45));

    private final Translation2d fl_xy = new Translation2d(defaultSwerve.getKFrontLeftXPos(),
            defaultSwerve.getKFrontLeftYPos())
            .minus(cameraOffset);

    private final Rotation3d fl_rot = new Rotation3d(Degrees.zero(), CAMERA_PITCH, Degrees.of(45));

    private final Transform3d fl_robotToCamera = calcRobotToCam(fl_xy, CAMERA_HEIGHT_OFF_GROUND, fl_rot);
    private final Transform3d fr_robotToCamera = rotateAroundCenter(fl_robotToCamera,
            new Rotation3d(Rotation2d.fromDegrees(-90)));
    private final Transform3d rl_robotToCamera = rotateAroundCenter(fl_robotToCamera,
            new Rotation3d(Rotation2d.fromDegrees(90)));
    private final Transform3d rr_robotToCamera = rotateAroundCenter(fl_robotToCamera,
            new Rotation3d(Rotation2d.fromDegrees(180)));

    PhotonCameraPlus fl_camera = new PhotonCameraPlus("fl_camera", fl_robotToCamera);
    PhotonCameraPlus fr_camera = new PhotonCameraPlus("fr_camera", fr_robotToCamera);
    PhotonCameraPlus rl_camera = new PhotonCameraPlus("rl_camera", rl_robotToCamera);
    PhotonCameraPlus rr_camera = new PhotonCameraPlus("rr_camera", rr_robotToCamera);

    Optional<PhotonCameraPlus> center_camera = Optional.empty();

    @SuppressWarnings("unused")
    private VisionConfig config;

    public VisionSystem(VisionConfig config) {
        super(config);
        this.config = config;

        if (config.isCenterCameraAttached()) {
            center_camera = Optional.of(
                    new PhotonCameraPlus(
                            "center_camera",
                            /*
                            * NOTICE: I'm not sure which direction is which with respect to the robot, 
                            * so I assumed that the front-back direction is 'x' value, the left-right direction is the 'y' value,
                            * and pitch is the direction in which the camera was tilted. -Nathaniel
                            */
                            new Transform3d(
                                    Units.inchesToMeters(10.5), //This value is the offset of the camera from the center considered in relation to the front and back.
                                    Units.inchesToMeters(3), //This value is the offset of the camera from the center considered in relation to the left and right.
                                    Units.inchesToMeters(3.5),
                                    new Rotation3d(0, Units.degreesToRadians(45), 0))));
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

            center_camera.ifPresent(
                    cc -> {
                        cc.update();
                    });
        }
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
    public void setupBindings() {
    }

    @Override
    public void setupDefaultCommand() {
    }
}
