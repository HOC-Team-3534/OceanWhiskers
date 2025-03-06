package frc.hocLib.util;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

/**
 * CachedDouble allows for a value to only be checked once per periodic loop if it is called by
 * multiple methods. Periodic is run first, so the value will be updated before it is used in any
 * Triggers or Command
 */
public class CachedMap<T, V> implements Function<T, V>, Subsystem {

    private Map<T, V> map = new HashMap<>();
    private Function<T, V> canCall;

    public CachedMap(Function<T, V> canCall) {
        this.canCall = canCall;
        this.register();
    }

    @Override
    public void periodic() {
        map.clear();
    }

    @Override
    public V apply(T key) {
        if (map.containsKey(key)) return map.get(key);
        var value = canCall.apply(key);
        map.put(key, value);
        return value;
    }
}
