package frc.hocLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.DoubleSupplier;

/** From 254 lib imported from 1678-2024 Contains basic functions that are used often. */
public class Util {

    public static final double EPSILON = 1e-12;

    /** Prevent this class from being instantiated. */
    private Util() {}

    /** Limits the given input to the given magnitude. */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** Checks if the given input is within the range (min, max), both exclusive. */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static boolean inRange(DoubleSupplier v, DoubleSupplier min, DoubleSupplier max) {
        return v.getAsDouble() > min.getAsDouble() && v.getAsDouble() < max.getAsDouble();
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static final Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);
    public static final Trigger autoMode =
            new Trigger(DriverStation::isAutonomousEnabled)
                    .or(new Trigger(DriverStation::isAutonomous));
    public static final Trigger testMode = new Trigger(DriverStation::isTestEnabled);
    public static final Trigger disabled = new Trigger(DriverStation::isDisabled);
    public static final Trigger dsAttached = new Trigger(DriverStation::isDSAttached);
}
