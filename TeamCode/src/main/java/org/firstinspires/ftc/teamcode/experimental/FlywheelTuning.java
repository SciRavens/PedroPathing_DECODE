package org.firstinspires.ftc.teamcode.experimental;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * FLYWHEEL PANELS TUNER
 * * Instructions:
 * Connect to robot Wi-Fi, open browser to: http://192.168.43.1:8001
 * Open the "Configurables" tab to edit values in real time.
 * Open the "Graph" tab and select "Target Velocity" and "Actual Velocity" to see them chart!
 */
@Configurable // FTControl's specific annotation to expose variables to the web UI
@TeleOp(name="Flywheel Panels Tuner", group="Tuning")
public class FlywheelTuning extends OpMode {

    private DcMotorEx shooterMotor;
    public Robot robot;

    // --- TUNING VARIABLES ---
    // MUST be 'public static' for @Configurable to see and edit them!
    public static double targetVelocity = 0;
    public static double P = 0.0001;
    public static double kV = 0.0004;
    public static double kS = 0.02;

    // FTControl's specific telemetry pipeline
    private TelemetryManager panelsTelemetry;

    // Button debouncing state
    private boolean lastUp = false, lastDown = false;
    private boolean lastY = false, lastA = false;
    private boolean lastX = false, lastB = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        // --- 1. SET UP PANELS TELEMETRY ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        panelsTelemetry.addLine("Ready to tune! Connect to Panels on port 8001.");
        panelsTelemetry.update();
    }

    @Override
    public void loop() {
        double velocity = shooterMotor.getVelocity(); // Ticks per second

        // --- 2. GAMEPAD CONTROLS (Still works alongside Panels!) ---
        if (gamepad1.dpad_up && !lastUp) targetVelocity += 25;
        if (gamepad1.dpad_down && !lastDown) targetVelocity -= 25;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        if (gamepad1.y && !lastY) P += 0.00005;
        if (gamepad1.a && !lastA) P -= 0.00005;
        lastY = gamepad1.y;
        lastA = gamepad1.a;

        if (gamepad1.b && !lastB) kV += 0.00005;
        if (gamepad1.x && !lastX) kV -= 0.00005;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        if (gamepad1.right_bumper) kS += 0.001;
        if (gamepad1.left_bumper) kS -= 0.001;

        // --- 3. MATH & MOTOR CONTROL ---
        double error = targetVelocity - velocity;
        double feedback = error * P;
        double feedforward = (kV * targetVelocity) + kS;

        double totalPower = feedback + feedforward;

        if (targetVelocity <= 0) {
            shooterMotor.setPower(0);
        } else {
            shooterMotor.setPower(Math.max(0, Math.min(1.0, totalPower)));
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.startIntake();
        } else {
            robot.intake.stopIntake();
        }

        // --- 4. PANELS CHARTING TELEMETRY ---
        // The Panels Graph tool can only chart raw, unformatted numeric values!
        panelsTelemetry.addData("Target Velocity", targetVelocity);
        panelsTelemetry.addData("Actual Velocity", velocity);
        panelsTelemetry.addData("Error", error);

        panelsTelemetry.addLine("--- Tuning Values ---");
        panelsTelemetry.addData("P", P);
        panelsTelemetry.addData("kV", kV);
        panelsTelemetry.addData("kS", kS);
        panelsTelemetry.addData("Total Power", totalPower);

        // Push the telemetry frame to the web dashboard
        panelsTelemetry.update();

        // (Optional) Mirror basic info to the physical Driver Hub screen
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Actual", velocity);
        telemetry.update();
    }
}