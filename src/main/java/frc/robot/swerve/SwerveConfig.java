package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import lombok.Getter;

public class SwerveConfig {
    @Getter private SwerveDrivetrainConstants drivetrainConstants;
    @Getter private SwerveModuleConstants<?, ?, ?>[] modules;

    public SwerveConfig(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        this.drivetrainConstants = drivetrainConstants;
        this.modules = modules;
    }
}
