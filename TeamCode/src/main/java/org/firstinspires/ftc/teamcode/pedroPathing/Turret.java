package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Turret control class for continuous rotation servo
 * Supports both power-based control (for PID) and position tracking
 */
public class Turret {
    private final CRServo turretServo;
    private double currentPower = 0.0;

    // Optional: position tracking (if you add an encoder later)
    private double estimatedPosition = 0.0;
    private long lastUpdateTime = System.nanoTime();

    public Turret(CRServo turretServo) {
        this.turretServo = turretServo;
        this.turretServo.setPower(0.0);
    }

    /**
     * Set turret rotation power
     * @param power -1.0 (full left) to 1.0 (full right)
     */
    public void setTurretPower(double power) {
        this.currentPower = clamp(power, -1.0, 1.0);
        turretServo.setPower(this.currentPower);
    }

    /**
     * Stop the turret
     */
    public void stop() {
        setTurretPower(0.0);
    }

    /**
     * Get current power being sent to servo
     */
    public double getCurrentPower() {
        return currentPower;
    }

    /**
     * Get estimated position (requires encoder or time integration)
     * This is a simple time-based estimator - not accurate without encoder
     */
    public double getEstimatedPosition() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9;

        // Simple integration: position += power * time * scale_factor
        // Adjust scale factor based on your servo speed
        estimatedPosition += currentPower * dt * 90.0; // ~90 deg/sec estimate

        lastUpdateTime = currentTime;
        return estimatedPosition;
    }

    /**
     * Reset position estimator (call when you know turret is at center)
     */
    public void resetPosition() {
        estimatedPosition = 0.0;
        lastUpdateTime = System.nanoTime();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}