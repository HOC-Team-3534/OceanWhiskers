package frc.reefscape;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.hocLib.util.BiMap;
import frc.hocLib.util.Util;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
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

    public static final Distance ReefBranchBetweenBranches = Inches.of(12.938);

    public static final Distance ReefCenterToBranchTip = Inches.of(30.738);

    public static final Distance InsideDiameterOfCoral = Inches.of(4.25);
    public static final Distance OutsideDiameterOfBranch = Inches.of(1.66);

    public static final class Reef {

        public static final Translation2d center =
                new Translation2d(Inches.of(176.746), FIELD_WIDTH.div(2));

        public static final double faceLength = Units.inchesToMeters(36.792600);
    }

    @RequiredArgsConstructor
    public enum ReefBranch {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;

        @Getter ReefSide reefSide;
        Map<ReefLevel, Pose3d> rawScoringLocations = new HashMap<>();
        Map<ReefLevel, Pose3d> scoringLocations = new HashMap<>();

        static {
            for (var branch : ReefBranch.values()) {
                var reefSideOrdinal = branch.ordinal() / 2;
                branch.reefSide = ReefSide.values()[reefSideOrdinal];

                var adjustX = ReefCenterToBranchTip;
                var adjustY =
                        ReefBranchBetweenBranches.div(2)
                                .times(branch.getSide().equals(Side.Left) ? -1 : 1);
                Pose2d groundScorePose =
                        new Pose2d(
                                        Reef.center,
                                        Rotation2d.fromDegrees(-180 + (60 * reefSideOrdinal)))
                                .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero));
                for (var level : ReefLevel.values()) {
                    Pose3d branchPose =
                            new Pose3d(
                                    new Translation3d(
                                            groundScorePose.getMeasureX(),
                                            groundScorePose.getMeasureY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            level.pitch.in(Radians),
                                            groundScorePose.getRotation().getRadians()));

                    branch.rawScoringLocations.put(level, branchPose);
                }
            }
        }

        public enum Side {
            Left,
            Right
        }

        public Side getSide() {
            return ordinal() % 2 == 0 ? Side.Left : Side.Right;
        }

        public Distance getOffset() {
            return ReefBranchBetweenBranches.div(2).times(getSide().equals(Side.Left) ? 1 : -1);
        }

        public Map<ReefLevel, Pose3d> getScoringLocations() {
            if (scoringLocations.size() != 0) return scoringLocations;

            for (var raw : rawScoringLocations.entrySet()) {
                var location = raw.getValue();
                var flippedLocation2d =
                        Util.applyIfRedAlliance(location.toPose2d(), FlippingUtil::flipFieldPose);
                var allianceBasedPose =
                        new Pose3d(flippedLocation2d)
                                .transformBy(
                                        new Transform3d(
                                                0,
                                                0,
                                                location.getZ(),
                                                new Rotation3d(
                                                        location.getRotation().getX(),
                                                        location.getRotation().getY(),
                                                        0)));

                scoringLocations.put(raw.getKey(), allianceBasedPose);
            }

            return scoringLocations;
        }

        public Pose3d getScoringLocation(ReefLevel level) {
            return getScoringLocations().get(level);
        }

        public Pose3d getScoredCoral(ReefLevel level) {
            return getScoringLocation(level).transformBy(level.getScoredCoralTransform());
        }
    }

    @RequiredArgsConstructor
    public enum ReefLevel {
        L1(Inches.of(25.0), Degrees.zero(), new Transform3d()),
        L2(
                Inches.of(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625),
                Degrees.of(-35),
                new Transform3d(
                        Units.inchesToMeters(-4.625),
                        0.0,
                        InsideDiameterOfCoral.minus(OutsideDiameterOfBranch)
                                .div(2)
                                .unaryMinus()
                                .in(Meters),
                        Rotation3d.kZero)),
        L3(
                Inches.of(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625),
                Degrees.of(-35),
                new Transform3d(
                        Units.inchesToMeters(-4.625),
                        0.0,
                        InsideDiameterOfCoral.minus(OutsideDiameterOfBranch)
                                .div(2)
                                .unaryMinus()
                                .in(Meters),
                        Rotation3d.kZero)),
        L4(
                Inches.of(72),
                Degrees.of(-90),
                new Transform3d(
                        Units.inchesToMeters(-5),
                        0.0,
                        Units.inchesToMeters(0.5),
                        new Rotation3d(0.0, Units.degreesToRadians(15), 0.0)));

        @Getter final Distance height;
        @Getter final Angle pitch;
        @Getter final Transform3d scoredCoralTransform;
    }

    private static BiMap<Integer, ReefSide> reefSideBlueMap, reefSideRedMap;

    @RequiredArgsConstructor
    public enum ReefSide {
        AB(18, 7),
        CD(17, 8),
        EF(22, 9),
        GH(21, 10),
        IJ(20, 11),
        KL(19, 6);
        @Getter final int blueTag, redTag;
        @Getter ReefBranch leftBranch, rightBranch;

        static {
            reefSideBlueMap = new BiMap<>();
            reefSideRedMap = new BiMap<>();
            for (var reefSide : ReefSide.values()) {
                reefSide.leftBranch = ReefBranch.valueOf(reefSide.name().substring(0, 1));
                reefSide.rightBranch = ReefBranch.valueOf(reefSide.name().substring(1, 2));
                reefSideBlueMap.put(reefSide.blueTag, reefSide);
                reefSideRedMap.put(reefSide.redTag, reefSide);
            }
        }

        public ReefBranch getBranch(ReefBranch.Side side) {
            return side.equals(ReefBranch.Side.Left) ? leftBranch : rightBranch;
        }

        public int getTagId() {
            return Util.isRedAlliance() ? redTag : blueTag;
        }

        public static Optional<ReefSide> getSide(int tagId) {
            return Optional.ofNullable(
                    Util.isRedAlliance() ? reefSideRedMap.get(tagId) : reefSideBlueMap.get(tagId));
        }

        public Optional<SideOfField> getSideOfField() {
            switch (this) {
                case AB, GH:
                    return Optional.empty();
                case CD, EF:
                    return Optional.of(SideOfField.Right);
                case IJ, KL:
                    return Optional.of(SideOfField.Left);
            }
            throw new RuntimeException();
        }
    }

    private static BiMap<Integer, LoadingStation> loadingStationBlueMap, loadingStationRedMap;

    @RequiredArgsConstructor
    public enum LoadingStation {
        Left(13, 1),
        Right(12, 2);
        @Getter final int blueTag, redTag;

        public int getTagId() {
            return Util.isRedAlliance() ? redTag : blueTag;
        }

        static {
            loadingStationBlueMap = new BiMap<>();
            loadingStationRedMap = new BiMap<>();
            for (var station : LoadingStation.values()) {
                loadingStationBlueMap.put(station.blueTag, station);
                loadingStationRedMap.put(station.redTag, station);
            }
        }

        public static Optional<LoadingStation> getStation(int tagId) {
            return Optional.ofNullable(
                    Util.isRedAlliance()
                            ? loadingStationRedMap.get(tagId)
                            : loadingStationBlueMap.get(tagId));
        }

        public static LoadingStation fromSide(SideOfField sideOfField) {
            return sideOfField.equals(SideOfField.Left) ? Left : Right;
        }
    }

    public enum SideOfField {
        Left,
        Right;

        public SideOfField opposite() {
            return this.equals(Left) ? Right : Left;
        }
    }

    public static List<AprilTag> getAllianceReefTags() {
        return Util.isRedAlliance() ? RED_REEF_APRIL_TAGS : BLUE_REEF_APRIL_TAGS;
    }
}
