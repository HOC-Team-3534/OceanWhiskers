package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Drive extends Command {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2) // Add a 20% deadband
            .withRotationalDeadband(MaxAngularRate * 0.2)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    final SwerveDriveSubsystem swerveDrive;
    final XboxController controller1;

    public Drive(SwerveDriveSubsystem swerveDrive, XboxController controller1) {
        this.swerveDrive = swerveDrive;
        this.controller1 = controller1;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerveDrive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerveDrive
                        .applyRequest(() -> drive
                                .withVelocityX(-controller1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-controller1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-controller1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ));
    }
}
