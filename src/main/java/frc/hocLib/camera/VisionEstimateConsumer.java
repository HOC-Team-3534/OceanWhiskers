package frc.hocLib.camera;

import java.util.List;
import org.photonvision.EstimatedRobotPose;

public interface VisionEstimateConsumer<T extends Enum<T> & StdDevCategory<T>> {
    public void accept(EstimatedRobotPose estimate, T stdDevsCategory, List<Integer> tagIds);
}
