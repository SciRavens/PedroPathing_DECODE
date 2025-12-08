package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

@TeleOp(name = "RGBLight", group = "Examples")
public class RgbLightTest extends OpMode {
// 0.5 green
// 0.3 orange/red

    public Servo rgbLight;
    public Timer buttonTimer;
    private double cur_val = 0.0;
    private double display = 0.0;

    @Override
    public void init() {
        rgbLight = hardwareMap.get(Servo.class, "shooterLight");
        buttonTimer = new Timer();
    }

    @Override
    public void start() {
        rgbLight.setPosition(0.0);
        buttonTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && buttonTimer.getElapsedTime() > 250) {
            display += 0.1;
            buttonTimer.resetTimer();
        }
        if (gamepad1.dpad_down && buttonTimer.getElapsedTime() > 250) {
            display -= 0.1;
            buttonTimer.resetTimer();
        }

        if (buttonTimer.getElapsedTimeSeconds() > 0.5) {
            cur_val = (cur_val == 0) ? display : 0;
            buttonTimer.resetTimer();
        }
        rgbLight.setPosition(cur_val);
        telemetry.addData("CurLight value: ", cur_val);
        telemetry.update();
    }
}
