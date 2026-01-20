package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Vision {
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private Telemetry telemetry;

    // =========================================================
    // 1. PHYSICAL CONSTANTS (YOU MUST MEASURE THESE!)
    // =========================================================
    // Units: Inches and Degrees
    private static final double CAMERA_HEIGHT_INCHES = 13.5;      // Height of Limelight lens from floor
    private static final double TARGET_HEIGHT_INCHES = 29.0;      // Height of AprilTag center from floor
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;    // Angle of camera up from horizontal

    // =========================================================
    // 2. PID & FEEDFORWARD TUNING
    // =========================================================
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KD = 0.0035;

    // Friction Feedforward: Minimum power to start moving (Stiction)
    private static final double TURRET_K_STATIC = 0.15;

    // Chassis Feedforward: Counter-acts robot rotation.
    // Value = 1.0 / (Max Turret Velocity in Deg/Sec).
    // Approx 0.0004 for 435RPM motor.
    private static final double TURRET_HEADING_FF = 0.0004;

    // Centering PID (for resetting to 0)
    private static final double CENTER_KP = 0.035;
    private static final double CENTER_TOLERANCE = 2.0;

    // =========================================================
    // 3. SAFETY LIMITS
    // =========================================================
    private static final double MAX_LEFT_LIMIT = 180.0;
    private static final double MAX_RIGHT_LIMIT = -180.0;
    private static final double TURRET_MAX_POWER = 0.3;
    private static final double TURRET_DEADZONE = 0.5; // Degrees error to consider "aimed"

    // =========================================================
    // 4. STATE VARIABLES
    // =========================================================
    private double lastResultTimestamp = 0;   // Timestamp of last camera frame
    private double lastError = 0.0;         // For Derivative calc
    private ElapsedTime loopTimer = new ElapsedTime(); // For Odometry DT

    // Memory Aiming
    private double goalMapX = 0;            // Saved Field X of the goal
    private double goalMapY = 0;            // Saved Field Y of the goal
    private boolean hasTargetMemory = false; // Have we ever seen the tag?

    // Modes
    private boolean isCentering = false;    // Manual override to zero

    public Vision(HardwareMap hardwareMap, Robot robot, Follower follower, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(Robot.current_pipeline_id);
        this.limelight.start();

        this.follower = follower;
        this.turret = robot.turret;
        this.robot = robot;
        this.telemetry = telemetry;

        loopTimer.reset();
    }

    // --- PUBLIC CONTROL METHODS ---
    public void startCentering() { this.isCentering = true; }
    public void startTracking() { this.isCentering = false; }
    public void toggleCentering() { this.isCentering = !this.isCentering; }

    // --- MAIN UPDATE LOOP ---
    public void update() {
        // -----------------------------------------------------
        // 1. GATHER SENSOR DATA
        // -----------------------------------------------------
        // A. Robot Odometry (Pose & Velocity)
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeadingRad = robotPose.getHeading();

        double robotAngularVelRad = follower.getAngularVelocity();
        double robotAngularVelDeg = Math.toDegrees(robotAngularVelRad);

        // B. Turret State
        double currentTurretAngle = turret.getDegrees();

        // C. Limelight Data
        LLResult result = limelight.getLatestResult();
        boolean hasNewCameraFrame = false;
        double currentCamTimestamp = 0;

        if (result != null && result.isValid()) {
            currentCamTimestamp = result.getTimestamp();
            if (currentCamTimestamp != lastResultTimestamp) {
                hasNewCameraFrame = true;
            }
        }

        // -----------------------------------------------------
        // 2. DETERMINE ERROR & TARGET
        // -----------------------------------------------------
        double error = 0.0;
        double outputPower = 0.0;

        // --- MODE: FORCED CENTERING (Manual Reset) ---
        if (isCentering) {
            error = 0.0 - currentTurretAngle;
            // Simple P-Loop for centering
            if (Math.abs(error) > CENTER_TOLERANCE) {
                outputPower = (error * CENTER_KP) + (Math.signum(error) * TURRET_K_STATIC);
            }
            telemetry.addData("Vision/Mode", "CENTERING");
        }

        // --- MODE: TRACKING (Camera or Memory) ---
        else {
            boolean seeTag = false;
            double camTx = 0.0;
            double camTy = 0.0;

            // Check if we can see the tag right now
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == Robot.current_tag_id) {
                            camTx = fiducial.getTargetXDegrees();
                            camTy = fiducial.getTargetYDegrees();
                            seeTag = true;
                            break;
                        }
                    }
                }
            }

            // --- SUB-MODE A: VISIBLE (Live Tracking + Map Update) ---
            if (seeTag) {
                // 1. Update our "Mental Map" of where the goal is
                updateGoalCoordinates(robotX, robotY, robotHeadingRad, currentTurretAngle, camTx, camTy);

                // 2. Use the camera error directly (Most precise)
                error = camTx;

                // 3. Mark timestamp for DT calculation
                if (hasNewCameraFrame) {
                    double dt = (currentCamTimestamp - lastResultTimestamp) / 1000.0;
                    if(dt < 0.001) dt = 0.001; // Safety

                    // Update PID with new frame data
                    outputPower = calculatePID(error, dt);
                    lastResultTimestamp = currentCamTimestamp;
                } else {
                    // No new frame? Just hold previous power or apply Feedforward only
                    // For simplicity, we skip PID update this tiny sub-loop but apply FF below
                    outputPower = 0; // Will be overwritten by last loop's power or handled by FF
                }

                telemetry.addData("Vision/Mode", "LOCKED (Camera)");
            }

            // --- SUB-MODE B: BLIND (Memory Tracking via Odometry) ---
            else if (hasTargetMemory) {
                // Calculate where we *should* look based on stored Goal X/Y
                double distX = goalMapX - robotX;
                double distY = goalMapY - robotY;

                // Absolute angle on field needed to face goal
                double targetFieldAngle = Math.atan2(distY, distX);

                // Convert to Robot-Relative Turret Angle
                // Angle = Target - RobotHeading
                double relativeAngleRad = AngleUnit.normalizeRadians(robotHeadingRad - targetFieldAngle);
                double targetTurretAngle = Math.toDegrees(relativeAngleRad);

                error = targetTurretAngle - currentTurretAngle;

                // Handle wrapping (e.g. going from 179 to -179)
                while (error > 180) error -= 360;
                while (error <= -180) error += 360;

                // Check if target is physically reachable (Safety)
                boolean isReachable = (targetTurretAngle <= MAX_LEFT_LIMIT && targetTurretAngle >= MAX_RIGHT_LIMIT);
                if (!isReachable) {
                    error = 0; // Don't try to track through the deadzone
                }

                // Calculate PID using Loop Time (since Odometry updates fast)
                double dt = loopTimer.seconds();
                loopTimer.reset();
                if(dt > 0.1) dt = 0.02; // Cap DT if loop lagged

                outputPower = calculatePID(error, dt);

                telemetry.addData("Vision/Mode", "MEMORY (Odometry)");
            }

            // --- SUB-MODE C: IDLE ---
            else {
                outputPower = 0;
                telemetry.addData("Vision/Mode", "SEARCHING / IDLE");
            }

            // -----------------------------------------------------
            // 3. ADD CHASSIS FEEDFORWARD (Always Active in Tracking)
            // -----------------------------------------------------
            // If robot spins Left (+), Turret must spin Right (-) to stay locked.
            double headingFF = - (robotAngularVelDeg * TURRET_HEADING_FF);

            // Add to PID output
            if (!isCentering && (seeTag || hasTargetMemory)) {
                outputPower += headingFF;
            }
        }

        // -----------------------------------------------------
        // 4. FINAL OUTPUT SAFETY
        // -----------------------------------------------------
        // Check Left Limit
        if (currentTurretAngle >= MAX_LEFT_LIMIT && outputPower > 0) {
            outputPower = 0;
            telemetry.addData("Vision/Limit", "HIT LEFT");
        }
        // Check Right Limit
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && outputPower < 0) {
            outputPower = 0;
            telemetry.addData("Vision/Limit", "HIT RIGHT");
        }

        // Clamp Power
        outputPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, outputPower));

        // Deadband (Stop jitter when close and robot is still)
        if (!isCentering && Math.abs(outputPower) < 0.05 && Math.abs(robotAngularVelDeg) < 1.0 && Math.abs(error) < TURRET_DEADZONE) {
            outputPower = 0;
        }

        turret.setTurretPower(outputPower);

        // Telemetry
        telemetry.addData("Vision/Err", "%.2f", error);
        telemetry.addData("Vision/Power", "%.2f", outputPower);
        telemetry.addData("Vision/Goal", hasTargetMemory ? String.format("%.1f, %.1f", goalMapX, goalMapY) : "None");
    }

    // --- HELPER: UPDATE GOAL COORDINATES ---
    private void updateGoalCoordinates(double robotX, double robotY, double robotHeading, double turretAngle, double tx, double ty) {
        // 1. Calculate horizontal distance to target
        double angleToTargetDeg = CAMERA_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);
        double dist = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToTargetRad);

        // 2. Calculate absolute angle to target
        // AbsAngle = RobotHeading + TurretAngle + CameraOffset(tx)
        double absAngleRad = robotHeading + Math.toRadians(turretAngle) + Math.toRadians(tx);

        // 3. Polar to Cartesian
        goalMapX = robotX + (dist * Math.cos(absAngleRad));
        goalMapY = robotY + (dist * Math.sin(absAngleRad));

        hasTargetMemory = true;
    }

    // --- HELPER: CALCULATE PID ---
    private double calculatePID(double error, double dt) {
        // P Term
        double p = TURRET_KP * error;

        // D Term
        double derivative = (error - lastError) / dt;
        double d = TURRET_KD * derivative;

        // F Term (Static Friction)
        double f = 0;
        if (Math.abs(error) > TURRET_DEADZONE) {
            f = Math.signum(error) * TURRET_K_STATIC;
        }

        lastError = error;
        return p + d + f;
    }
}