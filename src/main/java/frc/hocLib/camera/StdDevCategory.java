package frc.hocLib.camera;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

public interface StdDevCategory<T extends Enum<T> & StdDevCategory<T>> {
    /** Returns whether this category fits the given vision estimate criteria. */
    boolean fitsCriteria(EstimatedRobotPose estimate, double avgTargetArea, double poseDifference);

    /** Returns the standard deviations for this category. */
    Vector<N3> getStdDevs();
}
