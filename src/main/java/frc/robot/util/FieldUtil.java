package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;
import static frc.reefscape.FieldAndTags2025.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.hocLib.util.Util;
import frc.reefscape.FieldAndTags2025.SideOfField;

public class FieldUtil {
    public static Pose2d flipIfRed(Pose2d pose) {
        return Util.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Translation2d flipIfRed(Translation2d translation) {
        return flipIfRed(new Pose2d(translation, Rotation2d.kZero)).getTranslation();
    }

    public static SideOfField getCurrentSide(Pose2d blueOriginCurrentPose) {
        return flipIfRed(blueOriginCurrentPose).getMeasureY().gt(FIELD_WIDTH.div(2))
                ? SideOfField.Left
                : SideOfField.Right;
    }

    private static boolean isRobotOnOurSide(Pose2d robotPose, Distance tolerance) {
        return flipIfRed(robotPose).getMeasureX().lte(FIELD_LENGTH.div(2).plus(tolerance));
    }

    public static boolean isRobotOnOurSide(Pose2d robotPose) {
        return isRobotOnOurSide(robotPose, Feet.of(1));
    }
}
