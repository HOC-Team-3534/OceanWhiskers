package frc.hocLib.mechanism;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import frc.hocLib.HocSubsystem;
import frc.hocLib.util.CachedValue;
import java.util.function.Supplier;
import lombok.Getter;

public abstract class Mechanism extends HocSubsystem {
    private final CachedValue<Voltage> cachedVoltage;
    private final CachedValue<Voltage> cachedSupplyVoltage;
    private final CachedValue<Current> cachedCurrent;
    private final CachedValue<Angle> cachedPosition;
    private final CachedValue<AngularVelocity> cachedVelocity;
    private final CachedValue<Double> cachedPercentage;
    private final CachedValue<Distance> cachedLinearPosition;

    private Config config;

    public Mechanism(Config config) {
        super(config);
        this.config = config;

        cachedVoltage = createCache(this::updateVoltage, Volts.zero());
        cachedSupplyVoltage = createCache(this::updateSupplyVoltage, Volts.zero());
        cachedCurrent = createCache(this::updateCurrent, Amps.zero());
        cachedPosition = createCache(this::updatePosition, Rotations.zero());
        cachedVelocity = createCache(this::updateVelocity, RotationsPerSecond.zero());
        cachedPercentage = createCache(this::updatePercentage, 0.0);
        cachedLinearPosition = createCache(this::updateLinearPosition, Meters.zero());
    }

    protected <T> CachedValue<T> createCache(Supplier<T> orgSupplier, T defValue) {
        return new CachedValue<T>(isAttached() ? orgSupplier : () -> defValue);
    }

    protected abstract Voltage updateVoltage();

    protected abstract Voltage updateSupplyVoltage();

    protected abstract Current updateCurrent();

    protected abstract Angle updatePosition();

    protected abstract AngularVelocity updateVelocity();

    protected abstract Double updatePercentage();

    protected abstract Distance updateLinearPosition();

    public Voltage getVoltage() {
        return cachedVoltage.get();
    }

    public Voltage getSupplyVoltage() {
        return cachedSupplyVoltage.get();
    }

    public Current getCurrent() {
        return cachedCurrent.get();
    }

    public Power getPower() {
        return getVoltage().times(getCurrent());
    }

    public Angle getPosition() {
        return cachedPosition.get();
    }

    public AngularVelocity getVelocity() {
        return cachedVelocity.get();
    }

    public double getPercentage() {
        return cachedPercentage.get();
    }

    public Distance getLinearPosition() {
        return cachedLinearPosition.get();
    }

    protected double positionToPercent(Angle position) {
        return position.minus(config.minPosition)
                .div(config.maxPosition.minus(config.minPosition))
                .magnitude();
    }

    protected double linearPositionToPercent(Distance position) {
        return position.minus(config.minLinearPosition)
                .div(config.maxLinearPosition.minus(config.minLinearPosition))
                .magnitude();
    }

    protected Angle percentToPosition(double percent) {
        return config.maxPosition.minus(config.minPosition).times(percent).plus(config.minPosition);
    }

    protected Distance percentToLinearPosition(double percent) {
        return config.maxLinearPosition
                .minus(config.minLinearPosition)
                .times(percent)
                .plus(config.minLinearPosition);
    }

    protected Angle linearPositionToPosition(Distance linearPosition) {
        return linearPositionToPosition(linearPosition, true);
    }

    protected Angle linearPositionToPosition(Distance linearPosition, boolean clip) {
        var percent = linearPositionToPercent(linearPosition);

        if (clip) {
            percent = Math.min(Math.max(percent, 0), 1);
        }

        return percentToPosition(percent);
    }

    protected abstract void setVoltageOut(Voltage voltage);

    @Override
    public String getName() {
        return config.getName();
    }

    public static class Config extends HocSubsystem.Config {
        @Getter private Distance minLinearPosition = Inches.zero();
        @Getter private Distance maxLinearPosition = Inches.one();

        @Getter private Angle minPosition = Rotations.zero();
        @Getter private Angle maxPosition = Rotations.one();

        public Config(String name) {
            super(name);
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
