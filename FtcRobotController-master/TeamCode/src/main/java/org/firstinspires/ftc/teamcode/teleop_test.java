package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.Sensors.distanceSensor;


@TeleOp(name = "teleop_test")
public class teleop_test extends LinearOpMode {
    private DcMotor motor;
    boolean motorCommandIssued = false; // Variabil
    private ColorSensorV3 colorSensorV3;
    private distanceSensor distance_sensor;
    private MagneticLimitSwitch limitSwitch;
    private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează hardware-ul
        colorSensorV3 = new ColorSensorV3(hardwareMap, "sensor_color");
       // motor = hardwareMap.get(DcMotor.class, "motor");
//        distance_sensor = new distanceSensor(hardwareMap, "sensor_distance");
//        limitSwitch = new MagneticLimitSwitch(hardwareMap, "limit_switch");
        servo=hardwareMap.get(Servo.class, "servo");
        // Resetează encoderul motorului
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        // Setează modul pentru a folosi encoderul
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            String detectedColor = colorSensorV3.detectColor();

            // Obține valorile brute RGB
            int[] rgbValues = colorSensorV3.getRawColors();

            // Afișează informațiile în telemetry
            telemetry.addData("Culoare detectată:", detectedColor);
            telemetry.addData("Red", rgbValues[0]);
            telemetry.addData("Green", rgbValues[1]);
            telemetry.addData("Blue", rgbValues[2]);
            telemetry.update();

            // Obține distanța în centimetri
//            double distanceCm = distance_sensor.getDistanceCm();
//
//            // Verifică dacă distanța este mai mică decât 10 cm
//            boolean isClose = distance_sensor.isCloserThan(10);
//
//            // Afișează informațiile pe telemetry
//            telemetry.addData("Distanță (cm):", distanceCm);
//            telemetry.addData("Este mai aproape de 10 cm?", isClose ? "DA" : "NU");
//            telemetry.update();
//
//            boolean isPressed = limitSwitch.isPressed();
//
//            // Afișează starea senzorului pe telemetry
//            telemetry.addData("Senzor activat?", isPressed ? "DA" : "NU");
//            telemetry.update();


//            if (isPressed) {
//                telemetry.addLine("Limit switch activat! Execut acțiunea.");
//                // Introdu logică, cum ar fi oprirea unui motor
//                // motor.setPower(0);
//            }

            
            //Obține valorile RAW
//            int red = colorSensor.red();
//            int green = colorSensor.green();
//            int blue = colorSensor.blue();
//
//            // Detectează culorile
//            String detectedColor = "Necunoscut";
//            if (red > green && red > blue) {
//                detectedColor = "Roșu";
//            } else if (red > 100 && green > 100 && blue < 50) { // praguri pentru galben
//                detectedColor = "Galben";
//            } else if (blue > red && blue > green) {
//                detectedColor = "Albastru";
//            }
//
//            // Afișează culoarea detectată
//            telemetry.addData("Culoare detectată:", detectedColor);
////            telemetry.addData("Red", red);
////            telemetry.addData("Green", green);
////            telemetry.addData("Blue", blue);
//           // telemetry.update();
//
//            double distanceMM = distanceSensor.getDistance(DistanceUnit.MM);
//
//            // Afișează distanța pe Driver Station
//            telemetry.addData("Distanță (mm)", distanceMM);
//            telemetry.update();
//            // Verifică dacă butonul square este apăsat și culoarea nu este "Roșu"
            if (gamepad1.square  && !detectedColor.equals("Roșu")) {
               servo.setPosition(0.5);
            }
            else if (detectedColor.equals("Roșu")){
                servo.setPosition(0);
            }
        }
    }
}
