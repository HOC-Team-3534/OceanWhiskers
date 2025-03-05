package frc.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.reefscape.FieldAndTags2025.SideOfField;
import java.util.Optional;
import lombok.Getter;

public enum ReefSide {
    // spotless:off
    AB, CD, EF, GH, IJ, KL;
    // spotless:on

    @Getter
    final PathPlannerPath fromStartLeft, fromStartRight, fromLeft, fromRight, toLeft, toRight;

    ReefSide() {
        fromStartLeft = loadPath("_FROM_START_LEFT", "_FROM_START");
        fromStartRight = loadPath("_FROM_START_RIGHT", "_FROM_START");
        toLeft = loadPath("_TO_LEFT");
        fromLeft = loadPath("_FROM_LEFT");
        toRight = loadPath("_TO_RIGHT");
        fromRight = loadPath("_FROM_RIGHT");
    }

    PathPlannerPath loadPath(String postFix, String backupPostFix) {
        PathPlannerPath path = loadPath(postFix);

        if (path != null) return path;

        return loadPath(backupPostFix);
    }

    PathPlannerPath loadPath(String postfix) {
        try {
            return PathPlannerPath.fromPathFile(name() + postfix);
        } catch (Exception e) {
        }

        return null;
    }

    Optional<SideOfField> getSideOfField() {
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
