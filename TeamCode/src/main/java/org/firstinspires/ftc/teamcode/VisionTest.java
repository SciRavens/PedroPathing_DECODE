package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Vision-based turret tracking using Limelight and PID control
 * Features: Smooth tracking, velocity limiting, low-pass filtering
 */
public class VisionTest {
    private Limelight3A limelight;
    private Turret turret;

    // Target configuration

    // Turret PID constants - START HERE for tuning
    private static final double TURRET_KP = 0.035;  // TUNE THIS FIRST: Increase until it oscillates, then back off
    private static final double TURRET_KI = 0.0;    // LEAVE AT 0 - Usually causes twitching!
    private static final double TURRET_KD = 0.012;  // TUNE SECOND: Increase to dampen oscillation
    private static final double TURRET_DEADZONE = 0.5; // Wider deadzone = less twitching when locked
    private static final double TURRET_DEADZONE_EXIT = 1.0; // Must exceed this to exit lock (hysteresis)
    private static final double TURRET_MAX_POWER = 0.7; // Cap maximum power for stability
    private static final double TURRET_MIN_POWER = 0.05; // Minimum power to overcome friction

    // Velocity limiting - prevents servo from quitting on fast turns
    private static final double TURRET_MAX_ACCELERATION = 1.5; // Max power change per loop

    // Low-pass filter for smoothing
    private static final double FILTER_ALPHA = 0.6; // Lower = more smoothing (0=all history, 1=no filtering)

    // PID state variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private long lastLoopTime = 0;
    private boolean isLocked = false; // Track if turret is locked on target
    private static final double INTEGRAL_LIMIT = 0.3; // Prevent integral windup

    // Tracking state
    private boolean trackingTag = false;
    private int detectedTagID = -1;
    private double rawTx = 0.0;

    public VisionTest(HardwareMap hardwareMap, Robot robot) {
        this.turret = robot.turret;

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Robot.current_pipeline_id);
        limelight.start();

        lastLoopTime = System.nanoTime();
    }

    /**
     * Main update loop - call this every cycle
     */
    public void update() {
        // Calculate loop time for derivative
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero

        // Get Limelight results
        LLResult result = limelight.getLatestResult();
        trackingTag = false;
        rawTx = 0.0;
        detectedTagID = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Look for our target tag
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == Robot.current_tag_id) {
                        rawTx = fiducial.getTargetXDegrees();
                        detectedTagID = fiducial.getFiducialId();
                        trackingTag = true;
                        break;
                    }
                }

                // If we didn't find target tag, record first detected tag for diagnostics
                if (!trackingTag && !fiducials.isEmpty()) {
                    detectedTagID = fiducials.get(0).getFiducialId();
                }
            }
        }

        if (trackingTag) {
            // Low-pass filter to smooth noisy measurements
            filteredTx = FILTER_ALPHA * rawTx + (1 - FILTER_ALPHA) * filteredTx;

            // PID calculation
            double error = filteredTx;

            // Proportional term
            double pTerm = TURRET_KP * error;

            // Integral term with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
            double iTerm = TURRET_KI * integralSum;

            // Derivative term (rate of change of error)
            double derivative = (error - lastError) / dt;
            double dTerm = TURRET_KD * derivative;

            // Combine PID terms
            double turretPower = pTerm + iTerm + dTerm;

            // Apply minimum power threshold to overcome friction
            if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
                turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
            }

            // Clamp to maximum power
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            // Velocity limiting - prevent sudden power changes
            double powerChange = turretPower - lastTurretPower;
            if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
                turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
            }

            // Apply deadzone with hysteresis (prevents twitching)
            if (isLocked) {
                // Once locked, need bigger error to unlock
                if (Math.abs(error) < TURRET_DEADZONE_EXIT) {
                    turret.setTurretPower(0);
                    integralSum = 0; // Reset integral when locked
                } else {
                    turret.setTurretPower(turretPower);
                    isLocked = false; // Lost lock due to large error
                }
            } else {
                // Not locked, check if we should lock
                if (Math.abs(error) < TURRET_DEADZONE) {
                    turret.setTurretPower(0);
                    integralSum = 0;
                    isLocked = true;
                } else {
                    turret.setTurretPower(turretPower);
                }
            }

            // Update state for next loop
            lastError = error;
            lastTurretPower = turretPower;

        } else {
            // Reset PID when target lost
            integralSum = 0;
            lastError = 0;
            filteredTx = 0;
            isLocked = false;
            turret.setTurretPower(0); // Stop turret when no target
        }
    }

    /**
     * Stop the turret and cleanup
     */
    public void stop() {
        if (turret != null) {
            turret.stopTurret();
        }
        if (limelight != null) {
            limelight.stop();
        }
    }

    // -------------------- GETTERS FOR TELEMETRY --------------------

    public boolean isTrackingTag() {
        return trackingTag;
    }

    public boolean isLocked() {
        return isLocked;
    }

    public int getDetectedTagID() {
        return detectedTagID;
    }

    public double getRawError() {
        return rawTx;
    }

    public double getFilteredError() {
        return filteredTx;
    }

    public double getTurretPower() {
        return lastTurretPower;
    }

    public String getStatus() {
        if (!trackingTag) {
            return "NO TARGET";
        } else if (isLocked) {
            return "LOCKED";
        } else {
            return "TRACKING";
        }
    }

    /**
     * Get diagnostic info for telemetry
     */
    public String getDiagnostics() {
        return String.format(
                "Mode: %s | Tag: %d | Raw: %.2f° | Filtered: %.2f° | Power: %.3f",
                getStatus(), detectedTagID, rawTx, filteredTx, lastTurretPower
        );
    }
}