package frc.hocLib.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class CachedSendableChooser<T> extends SendableChooser<T> {
    private final CachedValue<T> cachedSelection = new CachedValue<>(() -> super.getSelected());
    private boolean locked;
    private T lockedValue;

    public T getSelected() {
        return locked ? lockedValue : cachedSelection.get();
    }

    public void lock() {
        lockedValue = getSelected();
        locked = true;
    }

    public void unlock() {
        locked = false;
    }
}
