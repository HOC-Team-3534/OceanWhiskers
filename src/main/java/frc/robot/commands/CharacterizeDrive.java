package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CustomSwerveRequest.CharacterizeDriveMotors;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CharacterizeDrive extends Command {
    final CharacterizeDriveMotors request = new CharacterizeDriveMotors();
    final SwerveDriveSubsystem swerveDrive;
    final double quas_voltage, quas_duration;

    /**
     * 
     * @param quas_voltage
     *                      the quasiastic voltage per second
     * @param quas_duration
     *                      the quasiastic test duration
     */
    CharacterizeDrive(SwerveDriveSubsystem swerveDrive, double quas_voltage,
            double quas_duration) {
        this.swerveDrive = swerveDrive;
        this.quas_voltage = quas_voltage;
        this.quas_duration = quas_duration;
        this.addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();
        swerveDrive.setControl(request.withVoltageX(0));
        swerveDrive.resetCharacterizationData();
    }

    @Override
    public void execute() {
        swerveDrive.setControl(request
                .withVoltageX(swerveDrive.getTimeSinceStartCharacterizing().in(Seconds) * quas_voltage));
        var fl_motor = swerveDrive.getModule(0);
        var current_voltage_output = fl_motor.getDriveMotor().getMotorVoltage().getValue();
        var current_velocity = MetersPerSecond.of(fl_motor.getCurrentState().speedMetersPerSecond);
        swerveDrive.addCharacterizationData(current_voltage_output, current_velocity);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveDrive.setControl(new SwerveRequest.Idle());
        swerveDrive.printCharacterizationData();
    }

    @Override
    public boolean isFinished() {
        return swerveDrive.getTimeSinceStartCharacterizing().in(Seconds) > quas_duration;
    }
}
