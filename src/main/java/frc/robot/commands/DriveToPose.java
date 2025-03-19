// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.Logging;
import frc.hocLib.swerve.DriveSpeedsConsumer;
import frc.hocLib.swerve.FieldRelativeSpeedsSupplier;
import frc.hocLib.swerve.MaxKinematicsSupplier;
import frc.hocLib.swerve.RobotPoseSupplier;
import frc.hocLib.util.GeomUtil;
import frc.hocLib.util.LoggedTunableNumber;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

public class DriveToPose<
                T extends
                        Subsystem & RobotPoseSupplier & DriveSpeedsConsumer
                                & FieldRelativeSpeedsSupplier & MaxKinematicsSupplier>
        extends Command {
    private static final LoggedTunableNumber drivekP =
            new LoggedTunableNumber("DriveToPose/DrivekP");
    private static final LoggedTunableNumber driveClosekP =
            new LoggedTunableNumber("DriveToPose/DriveClosekP");
    private static final LoggedTunableNumber drivekPDistance =
            new LoggedTunableNumber("DriveToPose/DrivekPDistance");
    private static final LoggedTunableNumber driveClosekPDistance =
            new LoggedTunableNumber("DriveToPose/DriveClosekPDistance");
    private static final LoggedTunableNumber drivekD =
            new LoggedTunableNumber("DriveToPose/DrivekD");
    private static final LoggedTunableNumber thetakP =
            new LoggedTunableNumber("DriveToPose/ThetakP");
    private static final LoggedTunableNumber thetakD =
            new LoggedTunableNumber("DriveToPose/ThetakD");
    private static final LoggedTunableNumber driveMaxVelocity =
            new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
    private static final LoggedTunableNumber driveMaxVelocitySlow =
            new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
    private static final LoggedTunableNumber driveMaxAcceleration =
            new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
    private static final LoggedTunableNumber thetaMaxVelocity =
            new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
    private static final LoggedTunableNumber thetaMaxAcceleration =
            new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
    private static final LoggedTunableNumber driveTolerance =
            new LoggedTunableNumber("DriveToPose/DriveTolerance");
    private static final LoggedTunableNumber thetaTolerance =
            new LoggedTunableNumber("DriveToPose/ThetaTolerance");
    private static final LoggedTunableNumber ffMinRadius =
            new LoggedTunableNumber("DriveToPose/FFMinRadius");
    private static final LoggedTunableNumber ffMaxRadius =
            new LoggedTunableNumber("DriveToPose/FFMaxRadius");

    static {
        drivekP.initDefault(RobotBase.isReal() ? 0.8 : 15.0);
        driveClosekP.initDefault(4.0);
        drivekPDistance.initDefault(Units.inchesToMeters(12.0));
        driveClosekPDistance.initDefault(Units.inchesToMeters(3.0));
        drivekD.initDefault(0.0);
        thetakP.initDefault(4.0);
        thetakD.initDefault(0.0);
        driveMaxVelocity.initDefault(3.8);
        driveMaxAcceleration.initDefault(3.0);
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(8.0);
        driveTolerance.initDefault(0.03);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        ffMinRadius.initDefault(0.05);
        ffMaxRadius.initDefault(0.1);
    }

    private final Supplier<Pose2d> target;
    private final T drive;

    private final ProfiledPIDController driveController =
            new ProfiledPIDController(
                    0.0,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(0.0, 0.0),
                    Robot.getConfig().LoopPeriod.in(Seconds));
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    0.0,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(0.0, 0.0),
                    Robot.getConfig().LoopPeriod.in(Seconds));

    private Translation2d lastSetpointTranslation = new Translation2d();
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    @Getter private boolean running = false;
    private Supplier<Pose2d> robot;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    private DriveSpeedsConsumer output;

    public DriveToPose(T drive, Supplier<Pose2d> target) {
        this.drive = drive;
        this.target = target;

        this.output = drive;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);

        robot = drive::getPose;
    }

    public DriveToPose(T drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this(drive, target);
        this.robot = robot;
    }

    public DriveToPose(
            T drive,
            Supplier<Pose2d> target,
            Supplier<Translation2d> linearFF,
            DoubleSupplier omegaFF) {
        this(drive, target);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    public DriveToPose(
            T drive,
            Supplier<Pose2d> target,
            Supplier<Pose2d> robot,
            Supplier<Translation2d> linearFF,
            DoubleSupplier omegaFF) {
        this(drive, target, robot);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    private DriveToPose(
            T drive,
            Supplier<Pose2d> target,
            Supplier<Pose2d> robot,
            DriveSpeedsConsumer output,
            Supplier<Translation2d> linearFF,
            DoubleSupplier omegaFF) {
        this(drive, target, robot);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
        this.output = output;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = drive.getFieldRelativeSpeeds();
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(
                                        target.get()
                                                .getTranslation()
                                                .minus(currentPose.getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        running = true;

        var percentNormalVsClose =
                (MathUtil.clamp(driveErrorAbs, driveClosekPDistance.get(), drivekPDistance.get())
                                - driveClosekPDistance.get())
                        / (drivekPDistance.get() - driveClosekPDistance.get());

        var updatedDriveKp =
                drivekP.get() * percentNormalVsClose
                        + driveClosekP.get() * (1 - percentNormalVsClose);

        driveController.setP(updatedDriveKp);

        // Update from tunable numbers
        if (driveMaxVelocity.hasChanged(hashCode())
                || driveMaxVelocitySlow.hasChanged(hashCode())
                || driveMaxAcceleration.hasChanged(hashCode())
                || driveTolerance.hasChanged(hashCode())
                || thetaMaxVelocity.hasChanged(hashCode())
                || thetaMaxAcceleration.hasChanged(hashCode())
                || thetaTolerance.hasChanged(hashCode())
                || drivekP.hasChanged(hashCode())
                || drivekD.hasChanged(hashCode())
                || thetakP.hasChanged(hashCode())
                || thetakD.hasChanged(hashCode())) {
            driveController.setD(drivekD.get());
            driveController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            driveMaxVelocity.get(), driveMaxAcceleration.get()));
            driveController.setTolerance(driveTolerance.get());
            thetaController.setP(thetakP.get());
            thetaController.setD(thetakD.get());
            thetaController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
            thetaController.setTolerance(thetaTolerance.get());
        }

        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Calculate drive speed
        double currentDistance =
                currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler =
                MathUtil.clamp(
                        (currentDistance - ffMinRadius.get())
                                / (ffMaxRadius.get() - ffMinRadius.get()),
                        0.0,
                        1.0);
        driveErrorAbs = currentDistance;

        var controllerSetpointTranslation =
                driveController.getSetpoint().equals(driveController.getGoal())
                                && !atGoal()
                                && driveErrorAbs > driveClosekPDistance.get()
                        ? currentPose.getTranslation()
                        : lastSetpointTranslation;

        driveController.reset(
                controllerSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler
                        + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation =
                new Pose2d(
                                targetPose.getTranslation(),
                                currentPose
                                        .getTranslation()
                                        .minus(targetPose.getTranslation())
                                        .getAngle())
                        .transformBy(
                                GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                        .getTranslation();

        // Calculate theta speed
        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                        + thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                targetPose.getRotation().getRadians());
        thetaErrorAbs =
                Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        Translation2d driveVelocity =
                new Pose2d(
                                new Translation2d(),
                                currentPose
                                        .getTranslation()
                                        .minus(targetPose.getTranslation())
                                        .getAngle())
                        .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                        .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
        driveVelocity =
                driveVelocity.interpolate(
                        linearFF.get().times(drive.getMaxVelocity().in(MetersPerSecond)), linearS);
        thetaVelocity =
                MathUtil.interpolate(
                        thetaVelocity,
                        omegaFF.getAsDouble() * drive.getMaxAngularVelocity().in(RadiansPerSecond),
                        thetaS);

        // Command speeds
        output.driveWithSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(),
                        driveVelocity.getY(),
                        thetaVelocity,
                        currentPose.getRotation()));

        // Log data
        Logging.log("DriveToPose/DistanceMeasured", currentDistance);
        Logging.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logging.log("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logging.log("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logging.log(
                "DriveToPose/Setpoint",
                new Pose2d[] {
                    new Pose2d(
                            lastSetpointTranslation,
                            Rotation2d.fromRadians(thetaController.getSetpoint().position))
                });
        Logging.log("DriveToPose/Goal", new Pose2d[] {targetPose});
    }

    @Override
    public void end(boolean interrupted) {
        output.driveWithSpeeds(new ChassisSpeeds());
        running = false;
        // Clear logs
        Logging.log("DriveToPose/Setpoint", new Pose2d[] {});
        Logging.log("DriveToPose/Goal", new Pose2d[] {});
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    Trigger AtGoalTrigger = new Trigger(this::atGoal).debounce(0.1);

    @Override
    public boolean isFinished() {
        return atGoal() && AtGoalTrigger.getAsBoolean();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
                && Math.abs(driveErrorAbs) < driveTolerance
                && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }

    public DriveToPose<T> withOutput(DriveSpeedsConsumer output) {
        return new DriveToPose<>(drive, target, robot, output, linearFF, omegaFF);
    }
}
