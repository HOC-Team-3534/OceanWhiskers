package frc.hocLib.util;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class ImpactDetector {
    // Previous chassis speeds and time (in seconds)
    private ChassisSpeeds previousSpeeds;
    private double previousTime;

    private final LinearAcceleration impactThreshold;
    @Getter private ChassisSpeeds impactChassisSpeeds = new ChassisSpeeds();

    public ImpactDetector(LinearAcceleration impactThreshold) {
        // Initialize previous speeds to zero
        previousSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        previousTime = System.currentTimeMillis() / 1000.0;
        this.impactThreshold = impactThreshold;
        timeSinceImpact.restart();
    }

    Timer timeSinceImpact = new Timer();

    /**
     * Call this method periodically (e.g., every robot cycle).
     *
     * @param currentSpeeds the current chassis speeds (vx, vy, omega)
     * @return true if a hard collision is detected, false otherwise.
     */
    public boolean update(ChassisSpeeds currentSpeeds) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        double dt = currentTime - previousTime;

        // Guard against very small or zero dt values
        if (dt <= 0.0) {
            dt = 0.02; // Assume a 20ms cycle as a fallback.
        }

        // Calculate linear acceleration components (change in vx and vy)
        double ax = (currentSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond) / dt;
        double ay = (currentSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond) / dt;

        var prevSpeedsMagnitude =
                new Translation2d(
                                previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond)
                        .getNorm();
        var currSpeedsMagnitude =
                new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                        .getNorm();

        // Compute the magnitude of the acceleration vector
        double accelerationMagnitude = Math.sqrt(ax * ax + ay * ay);

        // Update stored speeds and time for next cycle
        previousSpeeds = currentSpeeds;
        previousTime = currentTime;

        var highAccel = accelerationMagnitude >= impactThreshold.in(MetersPerSecondPerSecond);

        var decelerated = currSpeedsMagnitude < prevSpeedsMagnitude;

        var impact = highAccel && decelerated;

        if (impact && timeSinceImpact.hasElapsed(0.5)) {
            impactChassisSpeeds = previousSpeeds;
            timeSinceImpact.restart();
        }

        // If the acceleration (deceleration) exceeds the threshold, flag as a hard impact.
        return impact;
    }
}
