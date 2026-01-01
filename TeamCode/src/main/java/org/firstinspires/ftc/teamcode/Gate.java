package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gate {
    public Servo gateServo;

    public boolean gateClosed;

    public Gate(HardwareMap hardwareMap, Telemetry telemetry) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateClose();
        gateClosed = true;
    }

    public void gateOpen () {
        gateServo.setPosition(0);
        gateClosed = false;
    }

    public void gateClose() {
        gateServo.setPosition(0.18);
        gateClosed = true;
    }

    public boolean gateStatus() {
        return gateClosed;
    }
}
