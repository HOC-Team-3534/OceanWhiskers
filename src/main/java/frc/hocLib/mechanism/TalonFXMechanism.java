package frc.hocLib.mechanism;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.hocLib.HocSubsystem;
import frc.hocLib.talon.TalonFXFactory;
import frc.hocLib.util.CanDeviceId;
import lombok.Getter;
import lombok.Setter;

public abstract class TalonFXMechanism extends Mechanism {
    @Getter protected TalonFX motor;
    @Getter protected TalonFX[] followerMotors;

    @Getter protected Config config;

    public TalonFXMechanism(Config config) {
        super(config);

        this.config = config;

        if (isAttached()) {
            motor = config.getId().toTalonFX();
            motor.getConfigurator().apply(config.getTalonConfig());

            followerMotors = new TalonFX[config.getFollowerConfigs().length];
            for (int i = 0; i < config.getFollowerConfigs().length; i++) {
                var followerConfig = config.getFollowerConfigs()[i];

                followerMotors[i] =
                        TalonFXFactory.createPermanentFollower(
                                followerConfig.getId(), motor, followerConfig.opposeLeader);
            }

            setInstantiated(true);
        }
    }

    @Override
    protected Voltage updateVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    protected Voltage updateSupplyVoltage() {
        return motor.getSupplyVoltage().getValue();
    }

    @Override
    protected Current updateCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    @Override
    protected Angle updatePosition() {
        return motor.getPosition().getValue();
    }

    @Override
    protected AngularVelocity updateVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    protected Double updatePercentage() {
        return positionToPercent(getPosition());
    }

    @Override
    protected Distance updateLinearPosition() {
        return percentToLinearPosition(getPercentage());
    }

    private VoltageOut voltageOut = new VoltageOut(0);

    @Override
    protected void setVoltageOut(Voltage voltage) {
        if (isAttached()) {
            motor.setControl(voltageOut.withOutput(voltage));
        }
    }

    public static class FollowerConfig extends HocSubsystem.Config {
        @Getter private CanDeviceId id;
        @Getter private boolean opposeLeader = false;

        public FollowerConfig(String name, int id, boolean opposeLeader) {
            this(name, id, "", opposeLeader);
        }

        public FollowerConfig(String name, int id, String canbus, boolean opposeLeader) {
            super(name);
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    public static class Config extends Mechanism.Config {
        @Getter private CanDeviceId id;
        @Getter @Setter private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        public Config(String name, int id) {
            this(name, id, "");
        }

        public Config(String name, int id, String canbus) {
            super(name);
            this.id = new CanDeviceId(id, canbus);
        }

        public void setFollowerConfigs(FollowerConfig... followerConfigs) {
            this.followerConfigs = followerConfigs;
        }

        public void setSlot0Configs(Slot0Configs slot0Configs) {
            this.talonConfig.Slot0 = slot0Configs;
        }

        public void setMMConfigs(MotionMagicConfigs mmConfigs) {
            this.talonConfig.MotionMagic = mmConfigs;
        }

        public void setMotorOutputConfigs(MotorOutputConfigs motorOutputConfigs) {
            this.talonConfig.MotorOutput = motorOutputConfigs;
        }
    }
}
