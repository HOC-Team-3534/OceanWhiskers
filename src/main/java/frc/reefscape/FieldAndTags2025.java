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
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Distance FIELD_LENGTH = Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldLength());

    public static final Distance FIELD_WIDTH = Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

    public static final List<AprilTag> SORTED_APRIL_TAGS =
            APRIL_TAG_FIELD_LAYOUT.getTags().stream().sorted((t1, t2) -> t1.ID - t2.ID).toList();

    public static final List<AprilTag> BLUE_REEF_APRIL_TAGS = SORTED_APRIL_TAGS.subList(16, 22);

    public static final List<AprilTag> RED_REEF_APRIL_TAGS = SORTED_APRIL_TAGS.subList(5, 11);

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
