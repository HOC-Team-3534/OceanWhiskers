package frc.reefscape;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;

public final class FieldAndTags2025 {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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

    public enum PickupSide {
        Left(1, 13),
        Right(2, 12);
        int redId, blueId;

        PickupSide(int redId, int blueId) {
            this.redId = redId;
            this.blueId = blueId;
        }

        public Optional<Integer> getID() {
            return DriverStation.getAlliance()
                    .map(alliance -> alliance.equals(Alliance.Red) ? redId : blueId);
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
