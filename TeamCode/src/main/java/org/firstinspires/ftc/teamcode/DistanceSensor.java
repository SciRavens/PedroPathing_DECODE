package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {

    private AnalogInput distanceSensor;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    public DistanceSensor(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(AnalogInput.class, "distanceSensor");
    }

    public boolean isClose() {
        double distanceMM = getDistance();

        if (distanceMM >= 1000) {
            return true;
        }
        return false;
    }

    public double getDistance() {
        // Read sensor voltage (0.0–3.3V)
        double volts = distanceSensor.getVoltage();
        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

        return distanceMM;
    }
}
