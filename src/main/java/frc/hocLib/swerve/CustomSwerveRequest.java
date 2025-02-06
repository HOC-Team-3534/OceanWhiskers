package frc.hocLib.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public interface CustomSwerveRequest {
    public static class CharacterizeDriveMotors implements SwerveRequest {

        /** The voltage to apply to the drive motors. */
        double VoltageX = 0;

        VoltageOut voltageOut = new VoltageOut(0);

        @Override
        public StatusCode apply(
                SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(
                        voltageOut.withOutput(Volts.of(VoltageX)),
                        new PositionVoltage(Degrees.of(0)));
            }

            return StatusCode.OK;
        }

        /**
         * Sets the voltage of the drive motors.
         *
         * @param voltageX Voltage of the drive motors
         * @return this request
         */
        public CharacterizeDriveMotors withVoltageX(double voltageX) {
            this.VoltageX = voltageX;
            return this;
        }
    }
}
