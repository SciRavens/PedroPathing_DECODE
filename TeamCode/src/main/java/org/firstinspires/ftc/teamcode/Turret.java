package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    private DcMotorEx turretMotor;
    private Telemetry telemetry;

    // --- MOTOR CONSTANTS ---
    // GoBilda 5203 Series (435 RPM)
    // Spec: 384.5 ticks per revolution at the output shaft
    private static final double TICKS_PER_REV_MOTOR = 384.5;

    // --- MECHANICAL CONFIGURATION ---
    // IMPORTANT: Change this if you have external gears/belts!
    // Example: If motor has a 24T pulley and turret has a 120T turntable:
    // Reduction = 120 / 24 = 5.0
    // If Direct Drive (Motor shaft is the pivot), leave as 1.0.
    private static final double EXTERNAL_GEAR_REDUCTION = 150.0/38.0;

    // Calculated Ticks Per Degree
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV_MOTOR * EXTERNAL_GEAR_REDUCTION) / 360.0;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the motor
        // Ensure your config file names this "turretMotor"
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        this.telemetry = telemetry;

        // --- MOTOR CONFIGURATION ---

        // 1. Zero Power Behavior: BRAKE is critical for turrets to hold position
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Direction: Standard FTC convention is Positive = Counter-Clockwise (Left)
        // If your turret moves Right when given positive power, change to REVERSE.
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. Reset Encoder: Assume the starting position is "0 Degrees" (Center)
        // NOTE: The robot MUST be physically centered before initializing!
        resetEncoder();

        // 4. Mode: Run Without Encoder allows us to use raw power for our Custom PID
        // (We read the encoder manually in the Vision class)
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the raw power to the turret motor.
     * @param power -1.0 (Full Right) to 1.0 (Full Left)
     */
    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }

    /**
     * Gets the current angle of the turret in degrees.
     * 0 = Center (Starting Position)
     * Positive (+) = Left
     * Negative (-) = Right
     */
    public double getDegrees() {
        double currentTicks = turretMotor.getCurrentPosition();
        return currentTicks / TICKS_PER_DEGREE;
    }

    /**
     * Resets the logical center of the turret to the current position.
     * Useful if the turret gets knocked out of alignment.
     */
    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Immediate safety stop.
     */
    public void stop() {
        turretMotor.setPower(0);
    }
}