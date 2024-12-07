package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorV3 {
    private ColorSensor colorSensor;

    // Constructor care primește hardwareMap și numele senzorului
    public ColorSensorV3(HardwareMap hardwareMap, String sensorName) {
        this.colorSensor = hardwareMap.get(ColorSensor.class, sensorName);
    }

    // Metodă pentru a obține culoarea detectată
    public String detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > green && red > blue) {
            return "Roșu";
        } else if (red > 100 && green > 100 && blue < 50) { // praguri pentru galben
            return "Galben";
        } else if (blue > red && blue > green) {
            return "Albastru";
        }
        return "Necunoscut";
    }

    // Metodă pentru a obține valorile RGB brute
    public int[] getRawColors() {
        return new int[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }
}
