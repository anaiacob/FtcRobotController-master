package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distanceSensor {
    private DistanceSensor distance_sensor;

    // Constructor care inițializează senzorul
    public distanceSensor(HardwareMap hardwareMap, String sensorName) {
        this.distance_sensor = hardwareMap.get(DistanceSensor.class, sensorName);
    }

    // Metodă pentru a obține distanța în centimetri
    public double getDistanceCm() {
        return distance_sensor.getDistance(DistanceUnit.CM);
    }

    // Metodă pentru a obține distanța în milimetri
    public double getDistanceMm() {
        return distance_sensor.getDistance(DistanceUnit.MM);
    }

    // Metodă pentru a verifica dacă distanța este mai mică decât un prag
    public boolean isCloserThan(double thresholdCm) {
        return getDistanceCm() < thresholdCm;
    }
}
