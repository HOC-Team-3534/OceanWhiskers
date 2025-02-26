package frc.reefscape;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.Util;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public final class FieldAndTags2025 {
    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2025ReefscapeWelded;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(APRIL_TAG_FIELD);

    public static final Distance FIELD_LENGTH = Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldLength());

    public static final Distance FIELD_WIDTH = Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

    public static final List<AprilTag> SORTED_APRIL_TAGS =
            APRIL_TAG_FIELD_LAYOUT.getTags().stream().sorted((t1, t2) -> t1.ID - t2.ID).toList();

    public static final List<AprilTag> BLUE_REEF_APRIL_TAGS = SORTED_APRIL_TAGS.subList(16, 22);

    public static final List<AprilTag> RED_REEF_APRIL_TAGS = SORTED_APRIL_TAGS.subList(5, 11);

    public static boolean masked;

    static {
        updateMasking();
    }

    public static boolean needsMaskingUpdate() {
        return (masked && DriverStation.isFMSAttached())
                || (!masked && !DriverStation.isFMSAttached());
    }

    public static void updateMasking() {
        if (needsMaskingUpdate()) {
            var defaultField = AprilTagFieldLayout.loadField(APRIL_TAG_FIELD);
            if (!DriverStation.isFMSAttached()) {
                // maskAprilTag(defaultField, APRIL_TAG_FIELD_LAYOUT, 13, 18);
                // maskAprilTag(defaultField, APRIL_TAG_FIELD_LAYOUT, 18, 17);
                // maskAprilTag(defaultField, APRIL_TAG_FIELD_LAYOUT, 17, 22);
                // maskAprilTag(defaultField, APRIL_TAG_FIELD_LAYOUT, 11, 21);

                masked = true;
            } else {
                APRIL_TAG_FIELD_LAYOUT.getTags().stream()
                        .forEach(
                                tag -> {
                                    tag.pose = defaultField.getTagPose(tag.ID).orElse(tag.pose);
                                });

                masked = false;
            }
        }
    }

    static void maskAprilTag(
            AprilTagFieldLayout defaultLayout,
            AprilTagFieldLayout maskedLayout,
            int realId,
            int fakeId) {
        var tag = maskedLayout.getTags().stream().filter(t -> t.ID == realId).findFirst();
        var newPose = defaultLayout.getTagPose(fakeId);

        if (tag.isEmpty() || newPose.isEmpty()) return;

        tag.get().pose = newPose.get();
    }

    @RequiredArgsConstructor
    public enum AllianceValues {
        Blue(13, 12, pose -> pose.getMeasureX(), pose -> pose.getMeasureY()),
        Red(
                1,
                2,
                pose -> FIELD_LENGTH.minus(pose.getMeasureX()),
                pose -> FIELD_WIDTH.minus(pose.getMeasureY()));
        @Getter final int leftPickupId, rightPickupId;
        final Function<Pose2d, Distance> distanceFromAllianceWall, distanceFromRightWall;

        public Distance getDistanceFromAllianceWall(Pose2d blueOriginPose) {
            return distanceFromAllianceWall.apply(blueOriginPose);
        }

        public Distance getDistanceFromRightWall(Pose2d blueOriginPose) {
            return distanceFromRightWall.apply(blueOriginPose);
        }

        public static AllianceValues fromAlliance(Alliance alliance) {
            return alliance.equals(Alliance.Blue) ? Blue : Red;
        }
    }

    public static Optional<AllianceValues> getAllianceValues() {
        return Optional.of(
                AllianceValues.fromAlliance(Util.isRedAlliance() ? Alliance.Red : Alliance.Blue));
    }

    public enum SideOfField {
        Left,
        Right;

        public SideOfField opposite() {
            return this.equals(Left) ? Right : Left;
        }

        public Optional<Integer> getPickUpID() {
            return getAllianceValues()
                    .map(a -> this.equals(Left) ? a.getLeftPickupId() : a.getRightPickupId());
        }

        public static Optional<SideOfField> getCurrentSide(Pose2d blueOriginCurrentPose) {
            return getAllianceValues()
                    .map(values -> values.getDistanceFromRightWall(blueOriginCurrentPose))
                    .map(
                            distance ->
                                    distance.gt(FIELD_WIDTH.div(2))
                                            ? SideOfField.Left
                                            : SideOfField.Right);
        }
    }

    public static List<AprilTag> getAllianceReefTags() {
        return Util.isRedAlliance() ? RED_REEF_APRIL_TAGS : BLUE_REEF_APRIL_TAGS;
    }

    public static boolean isRobotOnOurSide(Pose2d robotPose2d) {
        return isRobotOnOurSide(robotPose2d, Feet.of(1));
    }

    public static boolean isRobotOnOurSide(Pose2d robotPose, Distance tolerance) {
        var values = getAllianceValues();
        if (values.isEmpty()) return false;
        return values.get()
                .getDistanceFromAllianceWall(robotPose)
                .lte(FIELD_LENGTH.div(2).plus(tolerance));
    }

    private static CachedValue<Optional<Integer>> cachedClosestHumanPlayerStation =
            new CachedValue<>(FieldAndTags2025::updateClosestHumanPlayerStationID);
    private static CachedValue<Optional<Integer>> cachedClosestReef =
            new CachedValue<>(FieldAndTags2025::updateClosestReefID);

    private static Pose2d getPose() {
        return Robot.getSwerve().getState().Pose;
    }

    private static Optional<Integer> updateClosestHumanPlayerStationID() {
        if (getPose().getMeasureY().isNear(FIELD_WIDTH.div(2), Feet.of(1))
                || !isRobotOnOurSide(getPose())) return Optional.empty();

        return SideOfField.getCurrentSide(getPose()).flatMap(SideOfField::getPickUpID);
    }

    public static Optional<Integer> getClosestHumanPlayerStationID() {
        return cachedClosestHumanPlayerStation.get();
    }

    private static Distance findDistanceFromRobot(Pose3d tag) {
        return Meters.of(
                tag.getTranslation().toTranslation2d().getDistance(getPose().getTranslation()));
    }

    private static Optional<Integer> updateClosestReefID() {
        if (!isRobotOnOurSide(getPose()) || getAllianceReefTags().isEmpty())
            return Optional.empty();

        return Optional.of(
                getAllianceReefTags().stream()
                        .min(
                                (t1, t2) -> {
                                    var dist1 = findDistanceFromRobot(t1.pose);
                                    var dist2 = findDistanceFromRobot(t2.pose);

                                    return (int) Math.round(dist1.minus(dist2).in(Meters) * 1000);
                                })
                        .get()
                        .ID);
    }

    public static Optional<Integer> getClosestReefID() {
        return cachedClosestReef.get();
    }
}
