package frc.robot.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.swerve.Telemetry;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private final Telemetry logger;

    private boolean warmedUp;

    private final SwerveConfig config;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                config.getSwerveModuleConstants());
        this.config = config;

        var pathPlannerConfig = loadRobotConfig();

        pathPlannerConfig.ifPresent(
                cfg ->
                        AutoBuilder.configure(
                                this::getPose,
                                this::resetPose,
                                this::getRobotRelativeSpeeds,
                                this::driveWithSpeeds,
                                new PPHolonomicDriveController(
                                        new PIDConstants(3.0, 0.0, 0.0),
                                        new PIDConstants(3.0, 0.0, 0.0)),
                                cfg,
                                // TODO: Would it be easier / possible to just worry about the blue
                                // side and then flip everything for red alliance side?
                                () -> false,
                                this));

        logger = new Telemetry(config.getKSpeedAt12Volts().in(MetersPerSecond));

        registerTelemetry(logger::telemeterize);
    }

    Optional<RobotConfig> loadRobotConfig() {
        try {
            return Optional.of(RobotConfig.fromGUISettings());
        } catch (Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    Pose2d getPose() {
        return getState().Pose;
    }

    ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    void driveWithSpeeds(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    @Override
    public void periodic() {
        keepOperatorPerspectiveUpdated();
        if (!warmedUp) {
            var warmup = FollowPathCommand.warmupCommand().withName("Follow Path Warmup");
            warmup.schedule();
            warmedUp = true;
        }
    }

    public Optional<Rotation2d> getRobotDriveDirection() {
        var speeds = getState().Speeds;
        var vector =
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                        .rotateBy(getPose().getRotation());

        if (vector.getNorm() < 0.05) return Optional.empty();

        return Optional.of(vector.getAngle());
    }

    /**
     * Periodically try to apply the operator perspective. If we haven't applied the operator
     * perspective before, then we should apply it regardless of DS state. This allows us to correct
     * the perspective in case the robot code restarts mid-match. Otherwise, only check and apply
     * the operator perspective if the DS is disabled. This ensures driving behavior doesn't change
     * until an explicit disable event occurs during testing.
     */
    void keepOperatorPerspectiveUpdated() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? Rotation2d.k180deg
                                                : Rotation2d.kZero);
                                m_hasAppliedOperatorPerspective = true;
                            });
        }
    }

    protected Command drive(
            Supplier<Double> xInput, Supplier<Double> yInput, Supplier<Double> rotInput) {
        var idle = new SwerveRequest.Idle();

        /* Setting up bindings for necessary control of the swerve drive platform */
        var fieldCentric =
                new SwerveRequest.FieldCentric()
                        .withDriveRequestType(
                                DriveRequestType
                                        .OpenLoopVoltage); // Use open-loop control for drive motors
        return run(
                () -> {
                    var maxSpeed = config.getKSpeedAt12Volts().in(MetersPerSecond);
                    var x = xInput.get() * maxSpeed;
                    var y = yInput.get() * maxSpeed;
                    var rot = rotInput.get() * config.getKMaxAngularRate().in(RadiansPerSecond);
                    if (x + y + rot < 0.05) {
                        setControl(idle);
                    } else {
                        setControl(
                                fieldCentric
                                        .withVelocityX(x)
                                        .withVelocityY(y)
                                        .withRotationalRate(rot));
                    }
                });
    }

    public Field2d getField() {
        return logger.getField();
    }

    @Override
    public void setupDefaultCommand() {
        SwerveStates.setupDefaultCommand();
    }

    @Override
    public void setupBindings() {
        SwerveStates.setupBindings();
    }
}
