package frc.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.reefscape.FieldAndTags2025.ReefSide;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.SneakyThrows;

public class Paths {
    private static final Map<ReefSide, ReefSidePaths> reefSidePathMap = new HashMap<>();

    static {
        for (var reefSide : ReefSide.values()) {
            reefSidePathMap.put(reefSide, new ReefSidePaths(reefSide));
        }
    }

    public static ReefSidePaths getReefSidePaths(ReefSide reefSide) {
        return reefSidePathMap.get(reefSide);
    }

    public static class ReefSidePaths {
        @Getter final ReefSide reefSide;
        @Getter
        final PathPlannerPath fromStartLeft, fromStartRight, fromLeft, fromRight, toLeft, toRight;

        ReefSidePaths(ReefSide reefSide) {
            this.reefSide = reefSide;
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

        @SneakyThrows
        PathPlannerPath loadPath(String postfix) {
            try {
                return PathPlannerPath.fromPathFile(reefSide.name() + postfix);
            } catch (Exception e) {
                return null;
            }
        }
    }
}
