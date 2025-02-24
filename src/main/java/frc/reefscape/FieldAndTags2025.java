package frc.reefscape;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public enum ReefSide {
        Front(7, 18),
        FrontRight(8, 17),
        BackRight(9, 22),
        Back(10, 21),
        BackLeft(11, 20),
        FrontLeft(6, 19);
        int redID, blueID;

        ReefSide(int redID, int blueID) {
            this.redID = redID;
            this.blueID = blueID;
        }

        public Optional<Integer> getID() {
            return DriverStation.getAlliance()
                    .map(alliance -> alliance.equals(Alliance.Red) ? redID : blueID);
        }
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
        return DriverStation.getAlliance().map(AllianceValues::fromAlliance);
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
        return DriverStation.getAlliance()
                .map(
                        alliance ->
                                alliance.equals(Alliance.Red)
                                        ? RED_REEF_APRIL_TAGS
                                        : BLUE_REEF_APRIL_TAGS)
                .orElse(List.of());
    }
}
