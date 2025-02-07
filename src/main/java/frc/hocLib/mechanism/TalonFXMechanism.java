package frc.hocLib.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.hocLib.HocSubsystem;
import frc.hocLib.talon.TalonFXFactory;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.CanDeviceId;
import lombok.Getter;
import lombok.Setter;

public abstract class TalonFXMechanism extends HocSubsystem {
    @Getter protected TalonFX motor;
    @Getter protected TalonFX[] followerMotors;

    @Getter protected Config config;

    private final CachedValue<Voltage> cachedVoltage;
    private final CachedValue<Current> cachedCurrent;
    private final CachedValue<Angle> cachedPosition;
    private final CachedValue<AngularVelocity> cachedVelocity;
    private final CachedValue<Double> cachedPercentage;
    private final CachedValue<Distance> cachedLinearPosition;

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
        }

        cachedVoltage = new CachedValue<Voltage>(this::updateVoltage);
        cachedCurrent = new CachedValue<Current>(this::updateCurrent);
        cachedPosition = new CachedValue<Angle>(this::updatePosition);
        cachedVelocity = new CachedValue<AngularVelocity>(this::updateVelocity);
        cachedPercentage = new CachedValue<Double>(this::updatePercentage);
        cachedLinearPosition = new CachedValue<Distance>(this::updateLinearPosition);
    }

    void telemetryInit() {
        SmartDashboard.putData(this);
    }

    @Override
    public String getName() {
        return config.getName();
    }

    Voltage updateVoltage() {
        if (isAttached()) {
            return motor.getMotorVoltage().getValue();
        }
        return Volts.zero();
    }

    public Voltage getVoltage() {
        return cachedVoltage.get();
    }

    Current updateCurrent() {
        if (isAttached()) {
            return motor.getStatorCurrent().getValue();
        }
        return Amps.zero();
    }

    public Current getCurrent() {
        return cachedCurrent.get();
    }

    public Power getPower() {
        return getVoltage().times(getCurrent());
    }

    Angle updatePosition() {
        if (isAttached()) {
            return motor.getPosition().getValue();
        }
        return Rotations.zero();
    }

    public Angle getPosition() {
        return cachedPosition.get();
    }

    AngularVelocity updateVelocity() {
        if (isAttached()) {
            return motor.getVelocity().getValue();
        }
        return RotationsPerSecond.zero();
    }

    public AngularVelocity getVelocity() {
        return cachedVelocity.get();
    }

    double updatePercentage() {
        return positionToPercent(getPosition());
    }

    public double getPercentage() {
        return cachedPercentage.get();
    }

    Distance updateLinearPosition() {
        return percentToLinearPosition(getPercentage());
    }

    public Distance getLinearPosition() {
        return cachedLinearPosition.get();
    }

    double positionToPercent(Angle position) {
        return position.minus(config.minPosition)
                .div(config.maxPosition.minus(config.minPosition))
                .magnitude();
    }

    double linearPositionToPercent(Distance position) {
        return position.minus(config.minLinearPosition)
                .div(config.maxLinearPosition.minus(config.minLinearPosition))
                .magnitude();
    }

    Angle percentToPosition(double percent) {
        return config.maxPosition.minus(config.minPosition).times(percent).plus(config.minPosition);
    }

    Distance percentToLinearPosition(double percent) {
        return config.maxLinearPosition
                .minus(config.minLinearPosition)
                .times(percent)
                .plus(config.minLinearPosition);
    }

    protected Angle linearPositionToPosition(Distance linearPosition) {
        return linearPositionToPosition(linearPosition, true);
    }

    Angle linearPositionToPosition(Distance linearPosition, boolean clip) {
        var percent = linearPositionToPercent(linearPosition);

        if (clip) {
            percent = Math.min(Math.max(percent, 0), 1);
        }

        return percentToPosition(percent);
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

    public static class Config extends HocSubsystem.Config {
        @Getter private CanDeviceId id;
        @Getter @Setter private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        @Getter private Distance minLinearPosition = Inches.zero();
        @Getter private Distance maxLinearPosition = Inches.one();

        @Getter private Angle minPosition = Rotations.zero();
        @Getter private Angle maxPosition = Rotations.one();

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

        public Config configMaxLinearPosition(Distance max) {
            maxLinearPosition = max;
            if (!minLinearPosition.lt(maxLinearPosition)) {
                throw new RuntimeException("Min Position must be less than Max Position");
            }
            return this;
        }

        public Config configMinMaxLinearPositions(Distance min, Distance max) {
            if (!min.lt(max)) {
                throw new RuntimeException("Min Position must be less than Max Position");
            }
            minLinearPosition = min;
            maxLinearPosition = max;
            return this;
        }

        public Config configMaxPosition(Angle max) {
            maxPosition = max;
            if (!minPosition.lt(maxPosition)) {
                throw new RuntimeException("Min Position must be less than Max Position");
            }
            return this;
        }

        public Config configMinMaxPosition(Angle min, Angle max) {
            if (!min.lt(max)) {
                throw new RuntimeException("Min Position must be less than Max Position");
            }
            minPosition = min;
            maxPosition = max;
            return this;
        }
    }
}
