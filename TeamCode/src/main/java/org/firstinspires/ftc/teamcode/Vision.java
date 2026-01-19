package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Vision {
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private Telemetry telemetry;

    // --- GOAL COORDINATES (Update for your Alliance!) ---
    // Example: Into The Deep High Basket coordinates (approximate, adjust for your field setup)
    public static double GOAL_X = 144.0;
    public static double GOAL_Y = 144.0;

    // --- TUNING VALUES ---
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KD = 0.0035;
    private static final double TURRET_K_STATIC = 0.0;

    // Feedforward: This helps the motor keep up with the calculated motion profile
    private static final double TURRET_HEADING_FF = 0.0;

    // --- SAFETY LIMITS ---
    private static final double MAX_LEFT_LIMIT = 180.0;
    private static final double MAX_RIGHT_LIMIT = -180.0;
    private static final double TURRET_MAX_POWER = 1.0;
    private static final double TURRET_DEADZONE = 0.5;

    // State Variables
    private long lastResultTimestamp = 0;
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

    public void setGoalLocation(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    public void update() {
        // 1. GET ROBOT DATA
        // PedroPathing Poses are usually (X, Y, Heading in Radians)
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeadingRad = robotPose.getHeading();

        // Velocity for Feedforward
        double robotAngularVelRad = follower.getAngularVelocity();
        double robotAngularVelDeg = Math.toDegrees(robotAngularVelRad);

        double currentTurretAngle = turret.getDegrees();

        // ----------------------------------------
        // STEP 2: CALCULATE ODOMETRY TARGET (The "Blind" Aim)
        // ----------------------------------------
        // Calculate the vector from robot to goal
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        // Calculate absolute field angle to goal (in Radians)
        double angleToGoalRad = Math.atan2(dy, dx);

        // Convert to Robot-Relative Angle (The angle the turret SHOULD face)
        // Formula: Goal Angle - Robot Heading
        double relativeAngleRad = AngleUnit.normalizeRadians(angleToGoalRad - robotHeadingRad);
        double targetTurretAngle = Math.toDegrees(relativeAngleRad);

        // ----------------------------------------
        // STEP 3: FUSE WITH LIMELIGHT (The "Precision" Aim)
        // ----------------------------------------
        LLResult result = limelight.getLatestResult();
        boolean seeTarget = false;
        double finalError = 0.0;

        // Check Limelight
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // If we see the tag, TRUST THE CAMERA over the Odometry
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == Robot.current_tag_id) {
                        finalError = fiducial.getTargetXDegrees(); // Camera says "Turn X degrees"
                        seeTarget = true;
                        break;
                    }
                }
            }
        }

        if (seeTarget) {
            // CASE A: Camera sees the goal. Use precise camera error.
            // (We assume camera error is 0 when centered)
            // Error = Target(0) - Current(tx) -> effectively just tx
            // We use the tx directly as the error signal for the PID
            finalError = finalError;
        } else {
            // CASE B: Camera is blind. Use Odometry Target.
            // Error = Desired_Angle - Current_Angle
            finalError = targetTurretAngle - currentTurretAngle;

            // Handle wrap-around (e.g. if error is 350, it should be -10)
            while (finalError > 180) finalError -= 360;
            while (finalError <= -180) finalError += 360;
        }

        // ----------------------------------------
        // STEP 4: SAFETY CHECK (Soft Limits)
        // ----------------------------------------
        // If the calculated odometry target is physically impossible (e.g., behind the robot
        // and wires prevent rotation), we must not try to go there.

        // Check if the Target Angle itself is out of bounds
        // (Note: This logic prevents "taking the long way around" into a hard stop)
        boolean targetReachable = (targetTurretAngle <= MAX_LEFT_LIMIT && targetTurretAngle >= MAX_RIGHT_LIMIT);

        // If blind and target is unreachable, we could center, or just stop.
        if (!seeTarget && !targetReachable) {
            finalError = 0; // Don't try to track through the dead zone
        }

        // ----------------------------------------
        // STEP 5: PID CONTROL
        // ----------------------------------------

        // Calculate Time (dt) for Derivative
        long currentTimestamp = System.nanoTime();
        // Fallback dt if timestamps are weird (0.02s = 50hz)
        double dt = 0.02;

        // PID Calc
        double pTerm = TURRET_KP * finalError;
        // Simple derivative on loop error (works fine for odometry tracking)
        double derivative = (finalError - lastError) / dt;
        double dTerm = TURRET_KD * derivative;

        double fTerm = 0;
        if (Math.abs(finalError) > TURRET_DEADZONE) {
            fTerm = Math.signum(finalError) * TURRET_K_STATIC;
        }

        // Feedforward: Even with odometry, the KV term helps overcome the robot's rotation momentum
        // If robot turns left, turret needs to whip right.
        double headingFF = - (robotAngularVelDeg * TURRET_HEADING_FF);

        double outputPower = pTerm + dTerm + fTerm + headingFF;

        // ----------------------------------------
        // STEP 6: FINAL OUTPUT
        // ----------------------------------------

        // Soft Limits (Hard Stops)
        if (currentTurretAngle >= MAX_LEFT_LIMIT && outputPower > 0) outputPower = 0;
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && outputPower < 0) outputPower = 0;

        outputPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, outputPower));

        // Update State
        lastError = finalError;

        turret.setTurretPower(outputPower);

        telemetry.addData("Vision/Source", seeTarget ? "CAMERA (Precise)" : "ODOMETRY (Blind)");
        telemetry.addData("Vision/Target Angle", "%.1f", targetTurretAngle);
        telemetry.addData("Vision/Current Angle", "%.1f", currentTurretAngle);
        telemetry.addData("Vision/Error", "%.1f", finalError);
    }
}