package frc.hocLib.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.hocLib.HocSubsystem;
import frc.hocLib.talon.TalonSRXFactory;
import frc.hocLib.util.CachedValue;
import frc.hocLib.util.CanDeviceId;
import lombok.Getter;
import lombok.Setter;

public abstract class TalonSRXMechanism extends Mechanism {
    @Getter protected TalonSRX motor;
    @Getter protected TalonSRX[] followerMotors;

    @Getter protected Config config;

    private final CachedValue<Double> cachedPositionInSensorTicks;
    private final CachedValue<Double> cachedVelocityInSensorTicksPer100ms;

    public TalonSRXMechanism(Config config) {
        super(config);

        this.config = config;

        if (isAttached()) {
            motor = config.getId().toTalonSRX();
            motor.configAllSettings(config.getTalonConfig());

            followerMotors = new TalonSRX[config.getFollowerConfigs().length];
            for (int i = 0; i < config.getFollowerConfigs().length; i++) {
                var followerConfig = config.getFollowerConfigs()[i];

                followerMotors[i] =
                        TalonSRXFactory.createPermanentFollower(followerConfig.getId(), motor);
            }

            setInstantiated(true);
        }

        cachedPositionInSensorTicks = createCache(this::updatePositionInSensorTicks, 0.0);
        cachedVelocityInSensorTicksPer100ms =
                createCache(this::updateVelocityInSensorTicksPer100ms, 0.0);
    }

    void telemetryInit() {
        SmartDashboard.putData(this);
    }

    protected double updatePositionInSensorTicks() {
        return motor.getSelectedSensorPosition();
    }

    public double getPositionInSensorTicks() {
        return cachedPositionInSensorTicks.get();
    }

    protected double updateVelocityInSensorTicksPer100ms() {
        return motor.getSelectedSensorVelocity();
    }

    public double getVelocityInSensorTicksPer100ms() {
        return cachedVelocityInSensorTicksPer100ms.get();
    }

    @Override
    protected Voltage updateVoltage() {
        return Volts.of(motor.getMotorOutputVoltage());
    }

    @Override
    protected Voltage updateSupplyVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    protected Current updateCurrent() {
        return Amps.of(motor.getStatorCurrent());
    }

    @Override
    protected Angle updatePosition() {
        return positionInSensorTicksToPosition(getPositionInSensorTicks());
    }

    @Override
    protected AngularVelocity updateVelocity() {
        return velocityInSensorTicksPer100msToVelocity(getVelocityInSensorTicksPer100ms());
    }

    @Override
    protected Double updatePercentage() {
        return positionToPercent(getPosition());
    }

    @Override
    protected Distance updateLinearPosition() {
        return percentToLinearPosition(getPercentage());
    }

    protected double positionToPositionInSensorTicks(Angle position) {
        return position.in(Rotations) / config.SensorToMechanismRatio * config.TicksPerRotation;
    }

    protected Angle positionInSensorTicksToPosition(double positionInSensorTicks) {
        return Rotations.of(
                positionInSensorTicks / config.TicksPerRotation * config.SensorToMechanismRatio);
    }

    protected double velocityToVelocityInSensorTicksPer100ms(AngularVelocity velocity) {
        return velocity.in(RotationsPerSecond)
                / 10
                / config.SensorToMechanismRatio
                * config.TicksPerRotation;
    }

    protected AngularVelocity velocityInSensorTicksPer100msToVelocity(
            double velocityInSensorTicksPer100ms) {
        return RotationsPerSecond.of(
                velocityInSensorTicksPer100ms
                        * 10
                        / config.TicksPerRotation
                        * config.SensorToMechanismRatio);
    }

    @Override
    protected void setVoltageOut(Voltage voltage) {
        if (isAttached()) {
            motor.set(ControlMode.PercentOutput, voltageToPercentOutput(voltage));
        }
    }

    protected double voltageToPercentOutput(Voltage voltage) {
        return voltage.div(getSupplyVoltage()).magnitude();
    }

    public static class FollowerConfig extends HocSubsystem.Config {
        @Getter private CanDeviceId id;
        // If followers must oppose leader, reverse how the motor is wired

        public FollowerConfig(String name, int id) {
            super(name);
            this.id = new CanDeviceId(id);
        }
    }

    public static class Config extends Mechanism.Config {
        @Getter private CanDeviceId id;
        @Getter @Setter private TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();

        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        @Getter private double TicksPerRotation;

        @Getter
        private double SensorToMechanismRatio; // Ex. 5 rotations of sensor equals one rotation of
        // mechanism would be 0.2

        public Config(String name, int id) {
            this(name, id, 4096, 1);
        }

        public Config(String name, int id, double TicksPerRotation, double SensorToMechanismRatio) {
            super(name);
            this.id = new CanDeviceId(id);
            this.TicksPerRotation = TicksPerRotation;
            this.SensorToMechanismRatio = SensorToMechanismRatio;
        }

        public void setFollowerConfigs(FollowerConfig... followerConfigs) {
            this.followerConfigs = followerConfigs;
        }

        public void setSlot0Configs(SlotConfiguration slot0Configs) {
            this.talonConfig.slot0 = slot0Configs;
        }

        /**
         * @param curveStrength Zero to use trapezoidal motion during motion magic. [1,8] for
         *     S-Curve, higher value for greater smoothing.
         */
        public void setMMConfigs(
                AngularVelocity cruiseVelocity,
                AngularAcceleration acceleration,
                int curveStrength) {
            this.talonConfig.motionCruiseVelocity =
                    velocityToVelocityInSensorTicksPer100ms(cruiseVelocity);
            this.talonConfig.motionAcceleration =
                    accelerationToAccelerationInSensorTicksPer100msPerSecond(acceleration);
            this.talonConfig.motionCurveStrength = curveStrength;
        }

        private double accelerationToAccelerationInSensorTicksPer100msPerSecond(
                AngularAcceleration acceleration) {
            return velocityToVelocityInSensorTicksPer100ms(acceleration.times(Seconds.one()));
        }

        private double velocityToVelocityInSensorTicksPer100ms(AngularVelocity velocity) {
            return velocity.in(RotationsPerSecond)
                    / 10
                    / this.SensorToMechanismRatio
                    * this.TicksPerRotation;
        }
    }
}
