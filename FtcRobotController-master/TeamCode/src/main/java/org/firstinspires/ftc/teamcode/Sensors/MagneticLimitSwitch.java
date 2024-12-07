package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MagneticLimitSwitch {
    private DigitalChannel limitSwitch;

    // Constructor care inițializează senzorul
    public MagneticLimitSwitch(HardwareMap hardwareMap, String sensorName) {
        this.limitSwitch = hardwareMap.get(DigitalChannel.class, sensorName);
        this.limitSwitch.setMode(DigitalChannel.Mode.INPUT); // Setează modul de citire
    }

    // Metodă pentru a verifica dacă senzorul este activat
    public boolean isPressed() {
        return !limitSwitch.getState(); // Returnează `true` dacă circuitul este închis (activat)
    }
}
