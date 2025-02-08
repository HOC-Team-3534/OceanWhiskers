/** Special thanks to FRC team 3847 */
package frc.hocLib;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;

/*
 * Represents a specific RoboRIO, as a key for configurations.
 *
 * The serial numbers here can be found on the label on the back: add a leading zero.
 *
 * Please keep the ID strings in lexical order.
 *
 * Note that the ID string may change when you reflash the RoboRIO.
 * Based on: https://github.com/Team100/all24/blob/2a109b28467cfddcafb93c7fc85ef60b56a628a2/lib/src/main/java/org/team100/lib/config/Identity.java
 */

public enum Rio {
    TBOT_2025("00", false), // TODO: Get serial number
    PBOT_2025("03073eaa", false), // TODO: Get serial number
    CBOT_2025("00", false), // TODO: Get serial number
    SIM("", true), // e.g. test default or simulation
    UNKNOWN(null, true);

    private static final Map<String, Rio> IDs = new HashMap<>();

    static {
        for (Rio i : Rio.values()) {
            IDs.put(i.serialNumber, i);
        }
    }

    private static final Alert rioIdAlert = new Alert("RIO: ", AlertType.kInfo);
    private static final Alert rioIdUnknown = new Alert("UNKNOWN RIO: ", AlertType.kError);
    private static final Alert rio1alert = new Alert("RIO 1.0", AlertType.kWarning);

    public static final Rio id = checkID();

    public static final String CANIVORE = "*"; // Use the first CANivore bus found
    public static final String RIO_CANBUS = "rio";

    private final String serialNumber;
    private final boolean isRio2;

    private Rio(String serialNumber, boolean isRio2) {
        this.serialNumber = serialNumber;
        this.isRio2 = isRio2;
    }

    private static Rio checkID() {
        rioIdAlert.set(false);
        rioIdUnknown.set(false);
        String serialNumber = "";
        if (RobotBase.isReal()) {
            // Calling getSerialNumber in a vscode unit test
            // SEGVs because it does the wrong
            // thing with JNIs, so don't do that.
            serialNumber = RobotController.getSerialNumber();
            Telemetry.print("RIO SERIAL: " + serialNumber);
        } else {
            serialNumber = "";
        }

        if (IDs.containsKey(serialNumber)) {
            Rio id = IDs.get(serialNumber);
            rioIdAlert.setText("Rio: " + id.name());
            rioIdAlert.set(true);

            Telemetry.print("RIO NAME: " + id.name());
            if (id.isRio2) {
                rio1alert.set(true);
            }
            return id;
        }
        rioIdUnknown.setText("Unknown Rio: " + serialNumber);
        rioIdUnknown.set(true);
        return UNKNOWN;
    }

    public boolean isRio2() {
        return isRio2;
    }
}
