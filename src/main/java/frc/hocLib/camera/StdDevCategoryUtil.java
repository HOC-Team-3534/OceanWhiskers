package frc.hocLib.camera;

import org.photonvision.EstimatedRobotPose;

public class StdDevCategoryUtil {
    public static <E extends Enum<E> & StdDevCategory<E>> E selectCategory(
            Class<E> enumClass,
            EstimatedRobotPose estimate,
            double avgTargetArea,
            double poseDifference) {
        for (E category : enumClass.getEnumConstants()) {
            if (category.fitsCriteria(estimate, avgTargetArea, poseDifference)) {
                return category;
            }
        }
        throw new IllegalArgumentException("No valid category found for the provided inputs.");
    }
}
