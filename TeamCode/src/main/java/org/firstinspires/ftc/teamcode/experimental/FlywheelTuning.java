package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Flywheel Button Tuner")
public class FlywheelTuning extends OpMode {
    private DcMotorEx motor;

    // Starting values (tweak these if they are too far off)
    public double targetVelocity = 1200;
    public double P = 0.0001;
    public double kV = 0.0004;
    public double kS = 0.02;

    // Track button states to prevent accidental double-clicks
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double velocity = motor.getVelocity();

        // --- TUNING CONTROLS ---

        // Dpad UP/DOWN: Adjust Target Velocity (steps of 50)
        if (gamepad1.dpad_up && !lastUp) targetVelocity += 50;
        if (gamepad1.dpad_down && !lastDown) targetVelocity -= 50;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Y / A: Adjust P (steps of 0.00005)
        if (gamepad1.y && !lastY) P += 0.00005;
        if (gamepad1.a && !lastA) P -= 0.00005;
        lastY = gamepad1.y;
        lastA = gamepad1.a;

        // B / X: Adjust kV (steps of 0.00005)
        if (gamepad1.b && !lastB) kV += 0.00005;
        if (gamepad1.x && !lastX) kV -= 0.00005;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        // Bumpers: Adjust kS (steps of 0.005)
        if (gamepad1.right_bumper) kS += 0.001; // No debounce needed for small creep
        if (gamepad1.left_bumper) kS -= 0.001;

        // --- MOTOR LOGIC ---
        double error = targetVelocity - velocity;
        double feedback = error * P;
        double feedforward = (kV * targetVelocity) + kS;

        motor.setPower(feedback + feedforward);

        // --- TELEMETRY ---
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Dpad Up/Down", "Target Velocity");
        telemetry.addData("Y / A", "Increase/Decrease P");
        telemetry.addData("B / X", "Increase/Decrease kV");
        telemetry.addData("Bumpers", "Adjust kS");
        telemetry.addLine("--- VALUES ---");
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("ActualVel", velocity);
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("kS", "%.4f", kS);
        telemetry.update();
    }
}