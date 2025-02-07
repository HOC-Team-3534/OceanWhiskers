package frc.robot.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.hocLib.HocSubsystem;
import lombok.Getter;

public class SwerveConfig extends HocSubsystem.Config {

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    @Getter private CANBus kCANBus = new CANBus("Swerve", "./logs/example.hoot");

    @Getter private int kPigeonId = 22;
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    @Getter private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    @Getter
    private SwerveDrivetrainConstants DrivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withCANBusName(kCANBus.getName())
                    .withPigeon2Id(kPigeonId)
                    .withPigeon2Configs(pigeonConfigs);

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @Getter
    private Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(100)
                    .withKI(0)
                    .withKD(0.5)
                    .withKS(0.1)
                    .withKV(1.59)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @Getter private Slot0Configs driveGains = new Slot0Configs().withKP(0.1);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    @Getter private ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    @Getter private ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    @Getter
    private DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    @Getter
    private SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    @Getter private SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    @Getter private Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    @Getter private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();

    @Getter
    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a relatively low
                                    // stator current limit to help avoid brownouts without
                                    // impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));

    @Getter private CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    @Getter private double kCoupleRatio = 3.5714285714285716;

    @Getter private double kDriveGearRatio = 6.746031746031747;
    @Getter private double kSteerGearRatio = 12.8;
    @Getter private Distance kWheelRadius = Inches.of(2);

    @Getter private boolean kInvertLeftSide = false;
    @Getter private boolean kInvertRightSide = true;

    // These are only used for simulation
    @Getter private MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    @Getter private MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    @Getter private Voltage kSteerFrictionVoltage = Volts.of(0.2);
    @Getter private Voltage kDriveFrictionVoltage = Volts.of(0.2);

    @Getter private Distance kWheelBase = Inches.of(21.5);
    @Getter private Distance kTrackWidth = Inches.of(21.5);

    // Front Left
    @Getter private int kFrontLeftDriveMotorId = 1;
    @Getter private int kFrontLeftSteerMotorId = 3;
    @Getter private int kFrontLeftEncoderId = 2;
    @Getter private Angle kFrontLeftEncoderOffset = Rotations.of(0.27392578125);
    @Getter private boolean kFrontLeftSteerMotorInverted = false;
    @Getter private boolean kFrontLeftEncoderInverted = false;
    @Getter private Distance kFrontLeftXPos, kFrontLeftYPos;

    // Front Right
    @Getter private int kFrontRightDriveMotorId = 4;
    @Getter private int kFrontRightSteerMotorId = 6;
    @Getter private int kFrontRightEncoderId = 5;
    @Getter private Angle kFrontRightEncoderOffset = Rotations.of(0.27392578125);
    @Getter private boolean kFrontRightSteerMotorInverted = false;
    @Getter private boolean kFrontRightEncoderInverted = false;
    @Getter private Distance kFrontRightXPos, kFrontRightYPos;

    // Back Left
    @Getter private int kBackLeftDriveMotorId = 7;
    @Getter private int kBackLeftSteerMotorId = 9;
    @Getter private int kBackLeftEncoderId = 8;
    @Getter private Angle kBackLeftEncoderOffset = Rotations.of(-0.123779296875);
    @Getter private boolean kBackLeftSteerMotorInverted = false;
    @Getter private boolean kBackLeftEncoderInverted = false;
    @Getter private Distance kBackLeftXPos, kBackLeftYPos;

    // Back Right
    @Getter private int kBackRightDriveMotorId = 10;
    @Getter private int kBackRightSteerMotorId = 12;
    @Getter private int kBackRightEncoderId = 11;
    @Getter private Angle kBackRightEncoderOffset = Rotations.of(0.44580078125);
    @Getter private boolean kBackRightSteerMotorInverted = false;
    @Getter private boolean kBackRightEncoderInverted = false;
    @Getter private Distance kBackRightXPos, kBackRightYPos;

    //
    @Getter private SwerveModuleConstants<?, ?, ?> FrontLeft, FrontRight, BackLeft, BackRight;

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    @Getter private LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.96);

    @Getter private Distance kSwerveCircumference;
    @Getter private AngularVelocity kMaxAngularRate;

    public SwerveConfig() {
        super("Swerve");
        updateConfig();
    }

    public SwerveModuleConstants<?, ?, ?>[] getSwerveModuleConstants() {
        return new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
    }

    public SwerveConfig configDriveGains(double ks, double kv, double ka) {
        driveGains.withKS(ks).withKV(kv).withKA(ka);
        updateConfig();
        return this;
    }

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        kFrontLeftEncoderOffset = Rotations.of(frontLeft);
        kFrontRightEncoderOffset = Rotations.of(frontRight);
        kBackLeftEncoderOffset = Rotations.of(backLeft);
        kBackRightEncoderOffset = Rotations.of(backRight);
        updateConfig();
        return this;
    }

    private void updateConfig() {
        var ConstantCreator =
                new SwerveModuleConstantsFactory<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withWheelRadius(kWheelRadius)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                        .withSlipCurrent(kSlipCurrent)
                        .withSpeedAt12Volts(kSpeedAt12Volts)
                        .withDriveMotorType(kDriveMotorType)
                        .withSteerMotorType(kSteerMotorType)
                        .withFeedbackSource(kSteerFeedbackType)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(encoderInitialConfigs)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage);

        kFrontLeftXPos = kWheelBase.div(2);
        kFrontLeftYPos = kTrackWidth.div(2);

        kFrontRightXPos = kFrontLeftXPos;
        kFrontRightYPos = kTrackWidth.div(2).unaryMinus();

        kBackLeftXPos = kWheelBase.div(2).unaryMinus();
        kBackLeftYPos = kFrontLeftYPos;

        kBackRightXPos = kBackLeftXPos;
        kBackRightYPos = kFrontRightYPos;

        kSwerveCircumference =
                Meters.of(
                                new Translation2d(
                                                kFrontLeftXPos.in(Meters),
                                                kFrontLeftYPos.in(Meters))
                                        .getNorm())
                        .times(2 * Math.PI);

        kMaxAngularRate =
                RotationsPerSecond.of(kSpeedAt12Volts.div(kSwerveCircumference).magnitude());

        FrontLeft =
                ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId,
                        kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        kFrontLeftXPos,
                        kFrontLeftYPos,
                        kInvertLeftSide,
                        kFrontLeftSteerMotorInverted,
                        kFrontLeftEncoderInverted);

        FrontRight =
                ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId,
                        kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        kFrontRightXPos,
                        kFrontRightYPos,
                        kInvertRightSide,
                        kFrontRightSteerMotorInverted,
                        kFrontRightEncoderInverted);

        BackLeft =
                ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId,
                        kBackLeftDriveMotorId,
                        kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        kBackLeftXPos,
                        kBackLeftYPos,
                        kInvertLeftSide,
                        kBackLeftSteerMotorInverted,
                        kBackLeftEncoderInverted);

        BackRight =
                ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId,
                        kBackRightDriveMotorId,
                        kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        kBackRightXPos,
                        kBackRightYPos,
                        kInvertRightSide,
                        kBackRightSteerMotorInverted,
                        kBackRightEncoderInverted);
    }
}
