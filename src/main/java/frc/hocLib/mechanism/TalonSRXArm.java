package frc.hocLib.mechanism;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.hocLib.util.CachedValue;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.Value;

public abstract class TalonSRXArm extends TalonSRXMechanism {

    @Getter
    public static class Config extends TalonSRXMechanism.Config {
        private final List<ArmSlotConfig> slotConfigs = new ArrayList<>();
        private final Angle startingAngle;
        @Setter private boolean sensorPhase;

        public Config(
                String name,
                int id,
                double TicksPerRotation,
                double SensorToMechanismRatio,
                Angle startingAngle,
                boolean sensorPhase,
                ArmSlotConfig slotConfig0,
                ArmSlotConfig... nextSlotConfigs) {
            super(name, id, TicksPerRotation, SensorToMechanismRatio);
            setSlotConfigs(slotConfig0, nextSlotConfigs);
            this.startingAngle = startingAngle;
            this.sensorPhase = sensorPhase;
        }

        public void setSlotConfigs(ArmSlotConfig slotConfig0, ArmSlotConfig... nextSlotConfigs) {
            this.slotConfigs.clear();
            this.slotConfigs.add(slotConfig0);
            for (var slotConfig : nextSlotConfigs) {
                this.slotConfigs.add(slotConfig);
            }
        }
    }

    @Value
    @RequiredArgsConstructor
    public static class ArmSlotConfig {
        private double kS;
        private double kG;
        private double kP;
        private double kI;
        private double kD;
        private double
                kFinVoltsSecondsPerRadianOfMechanism; // volts / (rads / s) is same as volts * s /
        // rads

        public SlotConfiguration getSlotConfiguration(Config config) {
            var slotConfiguration = new SlotConfiguration();

            slotConfiguration.kP = kP;
            slotConfiguration.kI = kI;
            slotConfiguration.kD = kD;

            var _100msOverSeconds = 10.0 / 1.0;

            var radsMechOverRadsSensor = 1.0 / config.getSensorToMechanismRatio();

            var radsOverRotations = (2 * Math.PI) / 1.0;

            var rotationsOverEncoderTicks = 1.0 / config.getTicksPerRotation();

            var dutyCycleOverVolts = 1023.0 / 12.0;

            slotConfiguration.kF =
                    kFinVoltsSecondsPerRadianOfMechanism
                            * _100msOverSeconds
                            * radsMechOverRadsSensor
                            * radsOverRotations
                            * rotationsOverEncoderTicks
                            * dutyCycleOverVolts;

            return slotConfiguration;
        }
    }

    private final Config config;

    @Getter private int currentSlot = 0;

    @Setter private int requestedCurrentSlot = 0;

    private final CachedValue<Double> cachedClosedLoopTargetPosition;
    private final CachedValue<Double> cachedActiveTrajectoryPosition;
    private final CachedValue<Double> cachedActiveTrajectoryVelocity;

    public TalonSRXArm(Config config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            for (int i = 0; i < config.slotConfigs.size(); i++) {
                motor.configureSlot(config.slotConfigs.get(i).getSlotConfiguration(config), i, 50);
            }

            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            motor.setSensorPhase(config.sensorPhase);

            Timer.delay(0.02);

            motor.setSelectedSensorPosition(positionToPositionInSensorTicks(config.startingAngle));

            motor.selectProfileSlot(requestedCurrentSlot, 0);
        }

        cachedClosedLoopTargetPosition = createCache(this::updateClosedLoopTargetPosition, 0.0);
        cachedActiveTrajectoryPosition = createCache(this::updateActiveTrajectoryPosition, 0.0);
        cachedActiveTrajectoryVelocity = createCache(this::updateActiveTrajectoryVelocity, 0.0);
    }

    private double updateClosedLoopTargetPosition() {
        return getControlMode().equals(ControlMode.MotionMagic) ? motor.getClosedLoopTarget() : 0.0;
    }

    private double updateActiveTrajectoryPosition() {
        return getControlMode().equals(ControlMode.MotionMagic)
                ? motor.getActiveTrajectoryPosition()
                : 0.0;
    }

    private double updateActiveTrajectoryVelocity() {
        return getControlMode().equals(ControlMode.MotionMagic)
                ? motor.getActiveTrajectoryVelocity()
                : 0.0;
    }

    public Angle getClosedLoopTarget() {
        return positionInSensorTicksToPosition(cachedClosedLoopTargetPosition.get());
    }

    public Angle getActiveTrajectoryPosition() {
        return positionInSensorTicksToPosition(cachedActiveTrajectoryPosition.get());
    }

    public AngularVelocity getActiveTrajectoryVelocity() {
        return velocityInSensorTicksPer100msToVelocity(cachedActiveTrajectoryVelocity.get());
    }

    protected double calculateArbitaryFeedforwardVolts(double directionValue) {
        var currentSlotConfig = config.getSlotConfigs().get(currentSlot);
        return currentSlotConfig.kG * Math.cos(getPosition().in(Radians))
                + currentSlotConfig.kS * Math.signum(directionValue);
    }

    protected double calculateArbitaryFeedforwardVolts() {
        return calculateArbitaryFeedforwardVolts(getError().magnitude());
    }

    protected void setTargetPosition(Angle angle) {
        if (isAttached()) {
            keepSlotConfigurationUpdated();

            var position = positionToPositionInSensorTicks(angle);

            var arbitraryFF = calculateArbitaryFeedforwardVolts() / 12.0;

            motor.set(
                    TalonSRXControlMode.MotionMagic,
                    position,
                    DemandType.ArbitraryFeedForward,
                    arbitraryFF);
        }
    }

    private void keepSlotConfigurationUpdated() {
        if (requestedCurrentSlot != currentSlot
                && requestedCurrentSlot < config.getSlotConfigs().size()
                && requestedCurrentSlot >= 0) {
            if (isAttached()) {
                motor.selectProfileSlot(requestedCurrentSlot, 0);
            }
            currentSlot = requestedCurrentSlot;
        }
    }

    public Angle getError() {
        return getClosedLoopTarget().minus(getPosition());
    }
}
