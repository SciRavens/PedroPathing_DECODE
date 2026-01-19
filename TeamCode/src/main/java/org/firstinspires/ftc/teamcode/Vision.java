package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Vision {
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private Telemetry telemetry;

    // --- TUNING VALUES ---
    // Start with these. If it oscillates, lower KP. If it's sluggish, increase KP.
    private static final double TURRET_KP = 0.0; //0.045;
    private static final double TURRET_KD = 0.0; // 0.0035;

    // Minimum power to overcome friction (Kickstart)
    private static final double TURRET_K_STATIC = 0.0;// 0.15;

    // Chassis Feedforward: Counters robot rotation.
    // Set to 0.0 initially. Increase to ~0.01 - 0.05 to make turret stay still when robot spins.
    private static final double TURRET_HEADING_FF = 0.0004;

    // --- CENTERING CONSTANTS ---
    private static final double CENTER_KP = 0.035;
    private static final double CENTER_TOLERANCE = 2.0;

    // --- SAFETY LIMITS ---
    private static final double MAX_LEFT_LIMIT = 180.0;
    private static final double MAX_RIGHT_LIMIT = -180.0;
    private static final double TURRET_MAX_POWER = 1.0;
    private static final double TURRET_DEADZONE = 0.5;

    // State Variables
    private double lastResultTimestamp = 0;
    private double lastError = 0.0;
    private boolean isCentering = false;

    public Vision(HardwareMap hardwareMap, Robot robot, Follower follower, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(Robot.current_pipeline_id);
        this.limelight.start();

        this.follower = follower;
        this.turret = robot.turret;
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void startCentering() { this.isCentering = true; }
    public void startTracking() { this.isCentering = false; }
    public void toggleCentering() { this.isCentering = !this.isCentering; }

    public void update() {
        // 1. GET CHASSIS VELOCITY (PedroPathing)
        // PedroPathing returns Radians/Sec. We convert to Degrees/Sec.
        double robotAngularVelRad = follower.getAngularVelocity();
        double robotAngularVelDeg = Math.toDegrees(robotAngularVelRad);

        double currentTurretAngle = turret.getDegrees();
        double outputPower = 0;

        // ----------------------------------------
        // MODE A: AUTO-CENTERING
        // ----------------------------------------
        if (isCentering) {
            double error = 0.0 - currentTurretAngle; // Target is 0

            if (Math.abs(error) > CENTER_TOLERANCE) {
                outputPower = error * CENTER_KP;
                outputPower += Math.signum(error) * TURRET_K_STATIC;
            }
        }

        // ----------------------------------------
        // MODE B: VISION TRACKING
        // ----------------------------------------
        else {
            LLResult result = limelight.getLatestResult();
            boolean hasNewData = false;

            // Check validity and timestamp
            if (result != null && result.isValid()) {
                double currentTimestamp = result.getTimestamp();
                if (currentTimestamp != lastResultTimestamp) {
                    hasNewData = true;

                    // 1. EXTRACT ERROR
                    double currentError = 0.0;
                    boolean tagFound = false;
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    if (fiducials != null && !fiducials.isEmpty()) {
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            if (fiducial.getFiducialId() == Robot.current_tag_id) {
                                currentError = fiducial.getTargetXDegrees();
                                tagFound = true;
                                break;
                            }
                        }
                        // Fallback to any tag if specific ID not found
                        if (!tagFound) {
                            currentError = fiducials.get(0).getTargetXDegrees();
                            tagFound = true;
                        }
                    }

                    if (tagFound) {
                        // 2. CALCULATE DT (Seconds)
                        double dt = (currentTimestamp - lastResultTimestamp) / 1000.0;
                        if (dt <= 0) dt = 0.001;

                        // 3. PID CALCULATIONS
                        double derivative = (currentError - lastError) / dt;

                        double pTerm = TURRET_KP * currentError;
                        double dTerm = TURRET_KD * derivative;
                        double fTerm = 0;

                        if (Math.abs(currentError) > TURRET_DEADZONE) {
                            fTerm = Math.signum(currentError) * TURRET_K_STATIC;
                        }

                        // Combine PID
                        outputPower = pTerm + dTerm + fTerm;

                        // Update History
                        lastError = currentError;
                        lastResultTimestamp = currentTimestamp;

                        telemetry.addData("Vision/Err", "%.2f", currentError);
                    }
                }
            }

            // 4. FEEDFORWARD (Always Active in Tracking Mode)
            // Even if camera has no new frame, we counter-rotate based on chassis movement.
            // If robot turns Left (+), Turret adds Right (-) power.
            double headingFF = - (robotAngularVelDeg * TURRET_HEADING_FF);
            outputPower += headingFF;
        }

        // ----------------------------------------
        // SAFETY LIMITS (Hard Stops)
        // ----------------------------------------
        if (currentTurretAngle >= MAX_LEFT_LIMIT && outputPower > 0) {
            outputPower = 0;
            telemetry.addData("Vision/Limit", "LEFT MAX");
        }
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && outputPower < 0) {
            outputPower = 0;
            telemetry.addData("Vision/Limit", "RIGHT MAX");
        }

        // Output Clamp
        outputPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, outputPower));

        // Final Deadband (Prevent hum when idle)
        if (!isCentering && Math.abs(outputPower) < 0.05 && Math.abs(robotAngularVelDeg) < 1.0) {
            outputPower = 0;
        }

        turret.setTurretPower(outputPower);

        telemetry.addData("Vision/State", isCentering ? "CENTERING" : "TRACKING");
        telemetry.addData("Vision/Power", "%.2f", outputPower);
        telemetry.addData("Vision/Angle", "%.1f", currentTurretAngle);
    }
}