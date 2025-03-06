package frc.reefscape;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import frc.hocLib.util.BiMap;
import frc.hocLib.util.Util;
import java.util.List;
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

    public static final Distance ReefBranchBetweenBranches = Inches.of(13.5);

    @RequiredArgsConstructor
    public enum ReefBranch {
        A(ReefSide.AB),
        B(ReefSide.AB),
        C(ReefSide.CD),
        D(ReefSide.CD),
        E(ReefSide.EF),
        F(ReefSide.EF),
        G(ReefSide.GH),
        H(ReefSide.GH),
        I(ReefSide.IJ),
        J(ReefSide.IJ),
        K(ReefSide.KL),
        L(ReefSide.KL);

        @Getter final ReefSide reefSide;

        public enum Side {
            Left,
            Right
        }

        public Side getSide() {
            return this.name().charAt(0) % 2 == 0 ? Side.Right : Side.Left;
        }

        public Distance getOffset() {
            return ReefBranchBetweenBranches.div(2).times(getSide().equals(Side.Left) ? 1 : -1);
        }
    }

    private static final BiMap<Integer, ReefSide> reefSideBlueMap = new BiMap<>(),
            reefSideRedMap = new BiMap<>();

    public enum ReefSide {
        AB(18, 7, ReefBranch.A, ReefBranch.B),
        CD(17, 8, ReefBranch.C, ReefBranch.D),
        EF(22, 9, ReefBranch.E, ReefBranch.F),
        GH(21, 10, ReefBranch.G, ReefBranch.H),
        IJ(20, 11, ReefBranch.I, ReefBranch.J),
        KL(19, 6, ReefBranch.K, ReefBranch.L);
        @Getter final int blueTag, redTag;
        @Getter final ReefBranch leftBranch, rightBranch;

        ReefSide(int blueTag, int redTag, ReefBranch leftBranch, ReefBranch rightBranch) {
            this.blueTag = blueTag;
            this.redTag = redTag;
            this.leftBranch = leftBranch;
            this.rightBranch = rightBranch;
            reefSideBlueMap.put(blueTag, this);
            reefSideRedMap.put(blueTag, this);
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

    private static final BiMap<Integer, LoadingStation> loadingStationBlueMap = new BiMap<>(),
            loadingStationRedMap = new BiMap<>();

    public enum LoadingStation {
        Left(13, 1),
        Right(12, 2);
        @Getter final int blueTag, redTag;

        LoadingStation(int blueTag, int redTag) {
            this.blueTag = blueTag;
            this.redTag = redTag;
            loadingStationBlueMap.put(blueTag, this);
            loadingStationRedMap.put(redTag, this);
        }

        public int getTagId() {
            return Util.isRedAlliance() ? redTag : blueTag;
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
