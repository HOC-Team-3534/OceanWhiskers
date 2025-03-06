package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.hocLib.Logging;
import frc.hocLib.characterization.FeedForwardCharacterizer;
import frc.hocLib.swerve.CustomSwerveRequest;
import frc.hocLib.swerve.DriveSpeedsConsumer;
import frc.hocLib.swerve.FieldRelativeSpeedsSupplier;
import frc.hocLib.swerve.MaxKinematicsSupplier;
import frc.hocLib.swerve.RobotPoseSupplier;
import frc.hocLib.swerve.Telemetry;
import frc.hocLib.swerve.VisionMeasurementAdder;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.Util;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements VisionMeasurementAdder,
                RobotPoseSupplier,
                DriveSpeedsConsumer,
                FieldRelativeSpeedsSupplier,
                MaxKinematicsSupplier {

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private final Telemetry logger;

    private boolean warmedUp;

    private SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(null, Volts.of(4), null),
                    new SysIdRoutine.Mechanism(
                            this::characterizeDriveWithVoltage, this::logFLMotor, this));

    private CustomSwerveRequest.CharacterizeDriveMotors characterizeDriveMotors =
            new CustomSwerveRequest.CharacterizeDriveMotors();

    private final SwerveConfig config;

    @Getter private final RobotConfig pathPlannerRobotConfig;

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
                config,
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                100.0,
                config.getSwerveModuleConstants());
        this.config = config;

        var pathPlannerConfig = loadRobotConfig();

        if (pathPlannerConfig.isEmpty())
            throw new RuntimeException("Unable to load Robot Config from pathPlanner");

        pathPlannerConfig.ifPresent(
                cfg ->
                        AutoBuilder.configure(
                                this::getPose,
                                this::resetPose,
                                this::getRobotRelativeSpeeds,
                                this::driveWithSpeeds,
                                new PPHolonomicDriveController(
                                        new PIDConstants(10.0, 0.0, 0.0),
                                        new PIDConstants(10.0, 0.0, 0.0)),
                                cfg,
                                () -> Util.isRedAlliance(),
                                this));

        pathPlannerRobotConfig = pathPlannerConfig.get();

        logger = new Telemetry(config.getKSpeedAt12Volts().in(MetersPerSecond));

        registerTelemetry(logger::telemeterize);
    }

    @Override
    public void periodic() {
        keepOperatorPerspectiveUpdated();
        if (!warmedUp) {
            var warmup = FollowPathCommand.warmupCommand().withName("Follow Path Warmup");
            var findingWarmup = PathfindingCommand.warmupCommand().withName("Pathfinding Warmup");
            warmup.schedule();
            findingWarmup.schedule();
            warmedUp = true;
        }

        Logging.log("Swerve", this);
    }

    @Override
    public void simulationPeriodic() {
        updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    Optional<RobotConfig> loadRobotConfig() {
        try {
            return Optional.of(RobotConfig.fromGUISettings());
        } catch (Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    private CachedValue<SwerveDriveState> cachedState = new CachedValue<>(() -> super.getState());

    @Override
    public SwerveDriveState getState() {
        return cachedState.get();
    }

    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }

    ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(), getPose().getRotation());
    }

    public void driveWithSpeeds(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    void characterizeDriveWithVoltage(Voltage voltage) {
        setControl(characterizeDriveMotors.withVoltageX(voltage.in(Volts)));
    }

    void logFLMotor(SysIdRoutineLog log) {
        var fl = getModule(0);
        var fl_drive = fl.getDriveMotor();
        log.motor("FL")
                .voltage(fl_drive.getMotorVoltage().getValue())
                .linearVelocity(MetersPerSecond.of(fl.getCurrentState().speedMetersPerSecond))
                .linearPosition(Meters.of(fl.getPosition(true).distanceMeters));
    }

    public Optional<Rotation2d> getRobotDriveDirection() {
        var speeds = getState().Speeds;
        var vector =
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                        .rotateBy(getPose().getRotation());

        if (vector.getNorm() < 0.1) return Optional.empty();

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
            setOperatorPerspectiveForward(
                    Util.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
            m_hasAppliedOperatorPerspective = true;
        }
    }

    public LinearVelocity getMaxVelocity() {
        return config.getKSpeedAt12Volts();
    }

    public AngularVelocity getMaxAngularVelocity() {
        return config.getKMaxAngularRate();
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

        // TODO: limit drive speed in case of elevator up or other reasons

        return run(
                () -> {
                    var maxSpeed = config.getKSpeedAt12Volts().in(MetersPerSecond);
                    var x = xInput.get() * maxSpeed;
                    var y = yInput.get() * maxSpeed;
                    var rot = rotInput.get() * config.getKMaxAngularRate().in(RadiansPerSecond);
                    if (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(rot, 2)) < 0.0025) {
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    protected Command characterize(Voltage quasVoltage, Time quasDuration) {
        var request = new CustomSwerveRequest.CharacterizeDriveMotors();
        var characterizer = new FeedForwardCharacterizer();
        return new FunctionalCommand(
                () -> {
                    setControl(request.withVoltageX(0));
                    characterizer.start();
                },
                () -> {
                    var requestedVoltage =
                            quasVoltage.in(Volts) * characterizer.getTimeSinceStart().in(Seconds);
                    setControl(request.withVoltageX(requestedVoltage));
                    var flModule = getModule(0);
                    var voltage = flModule.getDriveMotor().getMotorVoltage().getValue();
                    var velocity =
                            MetersPerSecond.of(flModule.getCurrentState().speedMetersPerSecond);
                    characterizer.add(velocity, voltage);
                },
                (interrupted) -> {
                    setControl(request.withVoltageX(0));
                    characterizer.print();
                },
                () -> quasDuration.minus(characterizer.getTimeSinceStart()).gt(Seconds.of(0)),
                this);
    }

    public Command driveStraightForward(LinearVelocity velocity) {
        return run(
                () -> driveWithSpeeds(new ChassisSpeeds(velocity.in(MetersPerSecond), 0.0, 0.0)));
    }

    public boolean isMoving() {
        return Math.abs(getState().Speeds.vxMetersPerSecond)
                        > InchesPerSecond.of(6.0).in(MetersPerSecond)
                || Math.abs(getState().Speeds.vyMetersPerSecond)
                        > InchesPerSecond.of(6.0).in(MetersPerSecond)
                || Math.abs(getState().Speeds.omegaRadiansPerSecond)
                        > DegreesPerSecond.of(5.0).in(RadiansPerSecond);
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

    @Override
    public void disabledInit() {
        setControl(new SwerveRequest.ApplyRobotSpeeds());
    }
}
