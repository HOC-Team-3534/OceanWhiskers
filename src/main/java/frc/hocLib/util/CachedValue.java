package frc.hocLib.util;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * CachedDouble allows for a value to only be checked once per periodic loop if it is called by
 * multiple methods. Periodic is run first, so the value will be updated before it is used in any
 * Triggers or Command
 */
public class CachedValue<T> implements Supplier<T>, Subsystem {

    private boolean isCached;
    private T value;
    private Supplier<T> canCall;
    @Getter private static int totalCachedValues = 0;

    public CachedValue(Supplier<T> canCall) {
        this.canCall = canCall;
        totalCachedValues++;
        this.register();
    }

    @Override
    public void periodic() {
        isCached = false;
    }

    @Override
    public T get() {
        if (!isCached) {
            value = canCall.get();
            isCached = true;
        }
        return value;
    }
}
