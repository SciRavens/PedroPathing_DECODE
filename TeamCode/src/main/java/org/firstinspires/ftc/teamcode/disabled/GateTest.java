package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Gate;

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

        telemetry.addData("The Gate is Closed", gate.isGateClosed());
    }


}
