package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.swerve.Telemetry;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final Telemetry logger;

    private boolean warmedUp;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public SwerveDriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        // TODO: Does there need to be custom odometry standard devs and custom vision standard devs matrices?
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        setDefaultCommand(new Drive(this));

        var config = loadRobotConfig();

        config.ifPresent(cfg -> AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveWithSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(3.0, 0.0, 0.0),
                        new PIDConstants(3.0, 0.0, 0.0)),
                cfg,
                // TODO: Would it be easier / possible to just worry about the blue side and then flip everything for red alliance side?
                () -> false,
                this));

        logger = new Telemetry(MaxSpeed);

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

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        keepOperatorPerspectiveUpdated();
        if (!warmedUp) {
            var warmup = FollowPathCommand.warmupCommand();
            warmup.schedule();
            warmedUp = true;
        }
    }

    /**
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    void keepOperatorPerspectiveUpdated() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    public Field2d getField() {
        return logger.getField();
    }
}
