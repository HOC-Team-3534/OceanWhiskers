package frc.hocLib.util;

import java.util.AbstractMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class BiMap<K, V> extends AbstractMap<K, V> {
    // The forward mapping (key → value)
    private final Map<K, V> forward = new HashMap<>();
    // The reverse mapping (value → key)
    private final Map<V, K> backward = new HashMap<>();

    @Override
    public V put(K key, V value) {
        // If the key already exists, remove its previous inverse mapping.
        if (forward.containsKey(key)) {
            V oldValue = forward.get(key);
            backward.remove(oldValue);
        }
        // Likewise, if the value already exists, remove its previous forward mapping.
        if (backward.containsKey(value)) {
            K oldKey = backward.get(value);
            forward.remove(oldKey);
        }
        forward.put(key, value);
        backward.put(value, key);
        return value;
    }

    @Override
    public V get(Object key) {
        return forward.get(key);
    }

    @Override
    public V remove(Object key) {
        V removed = forward.remove(key);
        if (removed != null) {
            backward.remove(removed);
        }
        return removed;
    }

    @Override
    public void clear() {
        forward.clear();
        backward.clear();
    }

    @Override
    public Set<Entry<K, V>> entrySet() {
        return forward.entrySet();
    }

    // Additional helper method for reverse lookup:
    public K getKey(Object value) {
        return backward.get(value);
    }

    // Optionally, override containsValue to use the reverse map
    @Override
    public boolean containsValue(Object value) {
        return backward.containsKey(value);
    }
}
