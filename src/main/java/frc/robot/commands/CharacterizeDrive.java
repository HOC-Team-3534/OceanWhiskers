package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CustomSwerveRequest.CharacterizeDriveMotors;

public class CharacterizeDrive extends Command {
    final CharacterizeDriveMotors request = new CharacterizeDriveMotors();
    final CommandSwerveDrivetrain swerveDrivetrain;
    final double quas_voltage, quas_duration;

    /**
     * 
     * @param quas_voltage
     *                      the quasiastic voltage per second
     * @param quas_duration
     *                      the quasiastic test duration
     */
    CharacterizeDrive(CommandSwerveDrivetrain swerveDrivetrain, double quas_voltage,
            double quas_duration) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.quas_voltage = quas_voltage;
        this.quas_duration = quas_duration;
        this.addRequirements(swerveDrivetrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        swerveDrivetrain.setControl(request.withVoltageX(0));
        swerveDrivetrain.resetCharacterizationData();
    }

    @Override
    public void execute() {
        swerveDrivetrain.setControl(request
                .withVoltageX(swerveDrivetrain.getTimeSinceStartCharacterizing().in(Seconds) * quas_voltage));
        var fl_motor = swerveDrivetrain.getModule(0);
        var current_voltage_output = fl_motor.getDriveMotor().getMotorVoltage().getValue();
        var current_velocity = MetersPerSecond.of(fl_motor.getCurrentState().speedMetersPerSecond);
        swerveDrivetrain.addCharacterizationData(current_voltage_output, current_velocity);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveDrivetrain.setControl(new SwerveRequest.Idle());
        swerveDrivetrain.printCharacterizationData();
    }

    @Override
    public boolean isFinished() {
        return swerveDrivetrain.getTimeSinceStartCharacterizing().in(Seconds) > quas_duration;
    }
}
