package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.characterization.FeedForwardCharacterizer;
import frc.robot.utils.swerve.CustomSwerveRequest.CharacterizeDriveMotors;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CharacterizeDrive extends Command {
    final CharacterizeDriveMotors request = new CharacterizeDriveMotors();
    final SwerveDriveSubsystem swerveDrive;
    final double quas_voltage, quas_duration;
    final FeedForwardCharacterizer characterizer = new FeedForwardCharacterizer();

    /**
     * 
     * @param quas_voltage
     *                      the quasiastic voltage per second
     * @param quas_duration
     *                      the quasiastic test duration
     */
    public CharacterizeDrive(SwerveDriveSubsystem swerveDrive, double quas_voltage,
            double quas_duration) {
        this.swerveDrive = swerveDrive;
        this.quas_voltage = quas_voltage;
        this.quas_duration = quas_duration;
        this.addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.setControl(request.withVoltageX(0));
        characterizer.start();
    }

    @Override
    public void execute() {
        var requested_voltage = characterizer.getTimeSinceStart().in(Seconds) * quas_voltage;
        swerveDrive.setControl(request.withVoltageX(requested_voltage));
        var fl_motor = swerveDrive.getModule(0);
        var voltage = fl_motor.getDriveMotor().getMotorVoltage().getValue();
        var velocity = MetersPerSecond.of(fl_motor.getCurrentState().speedMetersPerSecond);
        characterizer.add(velocity, voltage);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(new SwerveRequest.Idle());
        characterizer.print();
    }

    @Override
    public boolean isFinished() {
        return characterizer.getTimeSinceStart().in(Seconds) > quas_duration;
    }
}
