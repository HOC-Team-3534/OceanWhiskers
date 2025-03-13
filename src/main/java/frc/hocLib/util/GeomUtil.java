// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.hocLib.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.List;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(double x, double y) {
        return new Transform2d(x, y, new Rotation2d());
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Distance x, Distance y) {
        return new Transform2d(x, y, new Rotation2d());
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
                pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    public static Transform3d toTransform3d(Translation3d translation) {
        return new Transform3d(translation, Rotation3d.kZero);
    }

    public static Transform3d toTransform3d(Distance x, double y, double z) {
        return new Transform3d(new Translation3d(x.in(Meters), y, z), Rotation3d.kZero);
    }

    public static Transform3d toTransform3d(double x, Distance y, double z) {
        return new Transform3d(new Translation3d(x, y.in(Meters), z), Rotation3d.kZero);
    }

    public static Transform3d toTransform3d(double x, double y, Distance z) {
        return new Transform3d(new Translation3d(x, y, z.in(Meters)), Rotation3d.kZero);
    }

    public static Transform3d toTransform3d(Rotation3d rotation) {
        return new Transform3d(Translation3d.kZero, rotation);
    }

    public static Transform3d toTransform3d(Angle roll, double pitch, double yaw) {
        return new Transform3d(Translation3d.kZero, new Rotation3d(roll.in(Radians), pitch, yaw));
    }

    public static Transform3d toTransform3d(double roll, Angle pitch, double yaw) {
        return new Transform3d(Translation3d.kZero, new Rotation3d(roll, pitch.in(Radians), yaw));
    }

    public static Transform3d toTransform3d(double roll, double pitch, Angle yaw) {
        return new Transform3d(Translation3d.kZero, new Rotation3d(roll, pitch, yaw.in(Radians)));
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Creates a new pose from an existing one using a different translation value.
     *
     * @param pose The original pose
     * @param translation The new translation to use
     * @return The new pose with the new translation and original rotation
     */
    public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
        return new Pose2d(translation, pose.getRotation());
    }

    /**
     * Creates a new pose from an existing one using a different rotation value.
     *
     * @param pose The original pose
     * @param rotation The new rotation to use
     * @return The new pose with the original translation and new rotation
     */
    public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(pose.getTranslation(), rotation);
    }

    public static Distance calc2dDistance(Pose2d original, Pose3d other) {
        return Meters.of(
                other.getTranslation().toTranslation2d().getDistance(original.getTranslation()));
    }

    public static Transform3d averageTransform(List<Transform3d> transforms) {
        var n = transforms.size();
        if (n < 1) return new Transform3d();
        double x = 0.0, y = 0.0, z = 0.0, xRot = 0.0, yRot = 0.0, zRot = 0.0;
        for (var robotToCamera : transforms) {
            x += robotToCamera.getX();
            y += robotToCamera.getY();
            z += robotToCamera.getZ();
            xRot += robotToCamera.getRotation().getX();
            yRot += robotToCamera.getRotation().getY();
            zRot += robotToCamera.getRotation().getZ();
        }
        return new Transform3d(x / n, y / n, z / n, new Rotation3d(xRot / n, yRot / n, zRot / n));
    }

    /**
     * Calculates the intersection points of two circles in the XZ plane.
     *
     * @param center1 the center of the first circle (using x and z)
     * @param radius1 the radius of the first circle
     * @param center2 the center of the second circle (using x and z)
     * @param radius2 the radius of the second circle
     * @return a list of Translation3d representing the intersection points (0, 1, or 2 points)
     */
    public static List<Translation3d> calcIntercetionOfCirclesInXZPlane(
            Translation3d center1, Distance radius1, Translation3d center2, Distance radius2) {
        List<Translation3d> intersections = new ArrayList<>();
        double epsilon = 1e-9; // small tolerance for floating point comparisons

        // Extract x and z coordinates from the centers
        double x1 = center1.getX();
        double z1 = center1.getZ();
        double x2 = center2.getX();
        double z2 = center2.getZ();

        // Get the radii as double values
        double r1 = radius1.in(Meters);
        double r2 = radius2.in(Meters);

        // Compute the distance between centers in the XZ plane
        double dx = x2 - x1;
        double dz = z2 - z1;
        double d = Math.sqrt(dx * dx + dz * dz);

        // Check for no intersection conditions:
        // 1. Circles are too far apart.
        // 2. One circle is contained within the other.
        if (d > (r1 + r2) + epsilon || d < Math.abs(r1 - r2) - epsilon) {
            return intersections;
        }

        // Check for coincident circles (infinite intersections) and return empty.
        if (Math.abs(d) < epsilon && Math.abs(r1 - r2) < epsilon) {
            return intersections;
        }

        // Compute a (distance from center1 along the line between centers)
        double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        // Compute h (distance from the point along the line to the intersection points)
        double h = Math.sqrt(Math.max(0, r1 * r1 - a * a));

        // Determine the point (x3, z3) that is 'a' distance from center1 along the line between
        // centers
        double x3 = x1 + a * (dx / d);
        double z3 = z1 + a * (dz / d);

        // Use the y coordinate from center1 (adjust as needed)
        double y = center1.getY();

        // First intersection point
        double ix1 = x3 + h * (dz / d);
        double iz1 = z3 - h * (dx / d);
        Translation3d p1 = new Translation3d(ix1, y, iz1);
        intersections.add(p1);

        // If the circles are tangent, there is only one intersection
        if (Math.abs(d - (r1 + r2)) < epsilon || Math.abs(d - Math.abs(r1 - r2)) < epsilon) {
            return intersections;
        }

        // Second intersection point
        double ix2 = x3 - h * (dz / d);
        double iz2 = z3 + h * (dx / d);
        Translation3d p2 = new Translation3d(ix2, y, iz2);
        intersections.add(p2);

        return intersections;
    }
}
