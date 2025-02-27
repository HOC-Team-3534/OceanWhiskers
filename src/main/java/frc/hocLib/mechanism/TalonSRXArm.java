package frc.hocLib.mechanism;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import frc.hocLib.util.CachedValue;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public abstract class TalonSRXArm extends TalonSRXMechanism {

    @Getter
    @Setter
    @Accessors(chain = true)
    public static class Config extends TalonSRXMechanism.Config {
        private Supplier<ArmFeedforward> armFeedforward = () -> new ArmFeedforward(0.0, 0.0, 0.0);

        public Config(String name, int id) {
            super(name, id);
        }

        public Config(String name, int id, double TicksPerRotation, double SensorToMechanismRatio) {
            super(name, id, TicksPerRotation, SensorToMechanismRatio);
        }
    }

    private final Config config;

    private final CachedValue<Double> cachedClosedLoopTargetPosition;

    public TalonSRXArm(Config config) {
        super(config);
        this.config = config;

        cachedClosedLoopTargetPosition = createCache(motor::getClosedLoopTarget, 0.0);
    }

    public Angle getClosedLoopTarget() {
        return positionInSensorTicksToPosition(cachedClosedLoopTargetPosition.get());
    }

    public void setTargetPosition(Angle angle) {
        if (isAttached()) {
            var position = positionToPositionInSensorTicks(angle);

            var ff = config.getArmFeedforward().get();
            var arbitraryFF =
                    ff.getKg() * Math.cos(getPosition().in(Radians))
                            + ff.getKs()
                                    * Math.signum(
                                            getError()
                                                    .magnitude()); // TODO: calculate FF and convert
            // to percent output
            motor.set(
                    TalonSRXControlMode.MotionMagic,
                    position,
                    DemandType.ArbitraryFeedForward,
                    arbitraryFF);
        }
    }

    public Angle getError() {
        return getClosedLoopTarget().minus(getPosition());
    }
}
