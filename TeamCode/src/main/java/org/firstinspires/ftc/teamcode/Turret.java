package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    public DcMotorEx turretMotor;
    public TouchSensor centerLimitSwitch; // Added Limit Switch
    private Telemetry telemetry;

    // --- MOTOR CONSTANTS ---
    // GoBilda 5203 Series (435 RPM)
    // Spec: 384.5 ticks per revolution at the output shaft
    private static final double TICKS_PER_REV_MOTOR = 384.5;

    // --- MECHANICAL CONFIGURATION ---
    private static final double EXTERNAL_GEAR_REDUCTION = 150.0 / 38.0;

    // Calculated Ticks Per Degree
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV_MOTOR * EXTERNAL_GEAR_REDUCTION) / 360.0;
    // The new constant for Pedro Pathing math (2 * PI replaces 360)
    private static final double TICKS_PER_RADIAN = (TICKS_PER_REV_MOTOR * EXTERNAL_GEAR_REDUCTION) / (2 * Math.PI);

    public PIDFController turretPIDF;

    // --- CENTERING STATE MACHINE VARIABLES ---
    public enum TurretState {
        IDLE,
        SEARCHING
    }

    public enum SearchDirection {
        LEFT,
        RIGHT
    }

    private TurretState currentState = TurretState.IDLE;
    private ElapsedTime searchTimer = new ElapsedTime();

    private static final double SEARCH_POWER = 0.2; // Safe speed for homing
    private static final double MAX_SEARCH_TIME_SECONDS = 5.0; // Safety timeout

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        // IMPORTANT: Ensure your hardware map config matches this name!
        centerLimitSwitch = hardwareMap.get(TouchSensor.class, "turretLimitSwitch");

        this.telemetry = telemetry;

        // --- MOTOR CONFIGURATION ---
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretPIDF = new PIDFController(new PIDFCoefficients(1.5, 0.0, 0, 0.0));
    }


    /**
     * Triggers the non-blocking homing sequence.
     * @param hint The direction to turn to find the switch.
     */
    public void startCentering(SearchDirection hint) {
        if (currentState == TurretState.SEARCHING) {
            return; // Don't restart if already searching
        }

        // Based on your comments: Positive power = Left.
        // So if hint is LEFT, we apply positive SEARCH_POWER.
        double power = (hint == SearchDirection.LEFT) ? SEARCH_POWER : -SEARCH_POWER;

        setTurretPower(power);
        searchTimer.reset();
        currentState = TurretState.SEARCHING;
    }

    /**
     * MUST be called every cycle in your main OpMode loop.
     * Handles the limit switch logic without freezing the robot.
     */
    public void update() {
        if (currentState == TurretState.SEARCHING) {

            // 1. Check if the switch is pressed
            if (centerLimitSwitch.isPressed()) {
                stop();           // Immediately cut power
                resetEncoder();   // Zero the encoder
                currentState = TurretState.IDLE; // Return to normal operation
            }
            // 2. Safety timeout check
            else if (searchTimer.seconds() > MAX_SEARCH_TIME_SECONDS) {
                stop();           // Cut power to prevent wire damage
                currentState = TurretState.IDLE;
            }
        }

        telemetry.addData("TURRET: ", currentState);
        // You can also feed your custom PIDF controller inside this update
        // method in the future, if the state == IDLE!
    }

    /**
     * Use this in your OpMode to block normal auto-aim commands
     * while the turret is busy looking for its center.
     */
    public boolean isSearching() {
        return currentState == TurretState.SEARCHING;
    }

    // ==========================================================
    //                 STANDARD TURRET METHODS
    // ==========================================================

    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }

    public double getDegrees() {
        double currentTicks = turretMotor.getCurrentPosition();
        return currentTicks / TICKS_PER_DEGREE;
    }

    public double getTurretAngleRadians() {
        double currentTicks = turretMotor.getCurrentPosition();
        return currentTicks / TICKS_PER_RADIAN;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}