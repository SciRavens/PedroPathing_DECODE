package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Gate Test", group = "Examples")
public class GateTest extends OpMode {

    public Gate gate;

    @Override
    public void init() {
        gate = new Gate(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad2.b) {
            gate.gateOpen();
        } else if (gamepad2.x) {
            gate.gateClose();
        }

        telemetry.addData("The Gate is Closed", gate.gateStatus());
    }


}
