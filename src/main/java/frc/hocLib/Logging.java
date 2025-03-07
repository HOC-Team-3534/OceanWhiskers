package frc.hocLib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import dev.doglog.DogLog;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.hocLib.camera.StdDevCategory;
import frc.hocLib.mechanism.Mechanism;
import frc.hocLib.mechanism.TalonSRXArm;
import frc.hocLib.mechanism.TalonSRXMechanism;
import frc.robot.subsystems.swerve.Swerve;
import org.photonvision.EstimatedRobotPose;

public class Logging extends DogLog {

    public static void log(String key, PhotonCameraPlus<?> camera) {
        log(key + "/Connected", camera.isConnected());

        if (!camera.isConnected()) logFault("Camera Disconnected");
    }

    public static <T extends Enum<T> & StdDevCategory<T>> void log(
            String key, EstimatedRobotPose estimatedRobotPose, T category) {
        log(key + "/EstimatedRobotPose", estimatedRobotPose.estimatedPose.toPose2d());
        log(key + "/StdDevCategory", category.name());
        log(key + "/EstimateStdDevs", category.getStdDevs());
    }

    public static void log(String key, Mechanism mechanism) {
        log(key + "/Motor Voltage", mechanism.getVoltage().in(Volts));
        log(key + "/Supply Voltage", mechanism.getSupplyVoltage().in(Volts));
        log(key + "/Stator Current", mechanism.getStatorCurrent().in(Amps));
        log(key + "/Position (Degs.)", mechanism.getPosition().in(Degrees));
        log(key + "/Velocity (Degs. per Sec.)", mechanism.getVelocity().in(DegreesPerSecond));
        log(key + "/Percent of Range", mechanism.getPercentage());
        log(key + "/Linear Position (In.)", mechanism.getLinearPosition().in(Inches));
    }

    public static void log(String key, TalonSRXMechanism mechanism) {
        log(key + "/Duty Cycle", mechanism.getDutyCycleOut());
        log(key + "/Control Mode", mechanism.getControlMode());
        log(key, (Mechanism) mechanism);
    }

    public static void log(String key, TalonSRXArm arm) {
        log(key + "/Closed Loop Target (Degs.)", arm.getClosedLoopTarget().in(Degrees));
        log(
                key + "/Active Trajectory Position (Degs.)",
                arm.getActiveTrajectoryPosition().in(Degrees));
        log(
                key + "/Active Trajectory Velocity (Degs. per Sec.)",
                arm.getActiveTrajectoryVelocity().in(DegreesPerSecond));
        log(key, (TalonSRXMechanism) arm);
    }

    public static void log(String key, Swerve swerve) {
        log(key + "/Speed X", swerve.getState().Speeds.vxMetersPerSecond);
        log(key + "/Speed Y", swerve.getState().Speeds.vyMetersPerSecond);
        log(key + "/Speed Rotation", swerve.getState().Speeds.omegaRadiansPerSecond);
        log(key + "/Pose", swerve.getState().Pose);
    }
}
