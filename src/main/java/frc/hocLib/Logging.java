package frc.hocLib;

import static edu.wpi.first.units.Units.Volts;

import dev.doglog.DogLog;
import frc.hocLib.camera.PhotonCameraPlus;
import frc.hocLib.mechanism.Mechanism;

public class Logging extends DogLog {

    public static void log(String key, PhotonCameraPlus camera) {
        log(key + "/Connected", camera.isConnected());
    }

    public static void log(String key, Mechanism mechanism) {
        log(key + "/Motor Voltage", mechanism.getVoltage().in(Volts));
    }
}
