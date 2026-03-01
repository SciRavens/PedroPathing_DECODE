package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.disabled.SavePosition;

import java.util.Map;
import java.util.TreeMap;

public class TargetTracker {
    public SavePosition SavePosition;
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private Telemetry telemetry;

    // --- TUNING VALUES ---
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KD = 0.003;
    private static final double TURRET_MAX_POWER = 0.6;
    private static final double MAX_LEFT_LIMIT = Math.toRadians(95);
    private static final double MAX_RIGHT_LIMIT = Math.toRadians(-95);

    // --- VISION STATE & BUFFER ---
    private double smoothedAngularOffset = 0.0;
    private static final double OFFSET_EMA_ALPHA = 0.3;
    private long lastImageTimestamp = 0;

    // Buffer to store both Pose and Turret Angle
    private TreeMap<Long, RobotState> historyBuffer = new TreeMap<>();
    private static final int MAX_HISTORY_SIZE = 50;

    // Helper class to store a complete snapshot of the robot
    private static class RobotState {
        Pose pose;
        double turretAngle;

        RobotState(Pose pose, double turretAngle) {
            this.pose = pose;
            this.turretAngle = turretAngle;
        }
    }

    public TargetTracker(HardwareMap hardwareMap, Robot robot, Follower follower, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(robot.current_pipeline_id);
        this.limelight.start();
        this.follower = follower;
        this.turret = robot.turret;
        this.robot = robot;
        this.telemetry = telemetry;
        this.robot.turret.turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        // 1. Get current states
        long currentTimeMs = System.currentTimeMillis();
        Pose currentPose = follower.getPose();
        double currentTurretAngle = turret.getTurretAngleRadians();
        double robotHeading = currentPose.getHeading();

        // 2. Save the current state to the history buffer EVERY loop
        updateHistoryBuffer(currentTimeMs, currentPose, currentTurretAngle);

        // -----------------------------------------------------------------
        // STEP 1: ODOMETRY TARGETING (CURRENT)
        // -----------------------------------------------------------------
        double deltaX = robot.current_goal_x - currentPose.getX();
        double deltaY = robot.current_goal_y - currentPose.getY();
        double fieldTargetAngle = Math.atan2(deltaY, deltaX);

        // Where odometry thinks we should point right NOW
        double odomTargetTurretAngle = angleWrap(fieldTargetAngle - robotHeading);

        // -----------------------------------------------------------------
        // STEP 2: LATENCY-COMPENSATED VISION CORRECTION
        // -----------------------------------------------------------------
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Ensure we are only processing new frames
            long captureTimestamp = result.getControlHubTimeStamp();

            if (captureTimestamp > lastImageTimestamp) {
                lastImageTimestamp = captureTimestamp;

                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == robot.current_tag_id) {

                        // Calculate exactly when this image was taken
                        double latencyMs = result.getTargetingLatency() + result.getCaptureLatency();
                        long imageTimeMs = currentTimeMs - (long) latencyMs;

                        // Retrieve where the robot and turret were in the past
                        RobotState pastState = getHistoricalState(imageTimeMs);

                        if (pastState != null) {
                            // Reconstruct the past odometry geometry
                            double pastDeltaX = robot.current_goal_x - (pastState.pose.getX());
                            double pastDeltaY = robot.current_goal_y - pastState.pose.getY();
                            double pastFieldTargetAngle = Math.atan2(pastDeltaY, pastDeltaX);

                            // What odometry THOUGHT the target angle was in the past
                            double pastOdomTargetAngle = angleWrap(pastFieldTargetAngle - pastState.pose.getHeading());

                            // What vision ACTUALLY saw in the past
                            double txRadians = Math.toRadians(tag.getTargetXDegrees());
                            double truePastTargetAngle = pastState.turretAngle - txRadians;

                            // Calculate the offset based entirely on historical data
                            double rawOffset = angleWrap(truePastTargetAngle - pastOdomTargetAngle);

                            // Apply EMA smoothing to the calculated offset
                            smoothedAngularOffset = angleWrap(
                                    (OFFSET_EMA_ALPHA * rawOffset) + ((1.0 - OFFSET_EMA_ALPHA) * smoothedAngularOffset)
                            );
                        }
                        break;
                    }
                }
            }
        }


        // -----------------------------------------------------------------
        // STEP 3: APPLY OFFSET AND ENFORCE LIMITS
        // -----------------------------------------------------------------
        // Apply our latency-corrected, smoothed offset to our CURRENT odometry
        double finalTargetTurretAngle = angleWrap(odomTargetTurretAngle + smoothedAngularOffset);
        finalTargetTurretAngle = Math.max(MAX_RIGHT_LIMIT, Math.min(MAX_LEFT_LIMIT, finalTargetTurretAngle));

        // -----------------------------------------------------------------
        // STEP 4: PID & MOTOR COMMANDS
        // -----------------------------------------------------------------
        double error = angleWrap(finalTargetTurretAngle - currentTurretAngle);
        turret.turretPIDF.updateError(error);
        double power = turret.turretPIDF.run();

        // Safety Clamps
        if (currentTurretAngle >= MAX_LEFT_LIMIT && power > 0) power = 0;
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && power < 0) power = 0;

        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        turret.setTurretPower(power);

        // Telemetry
        telemetry.addData("Odom Target (Deg)", Math.toDegrees(odomTargetTurretAngle));
        telemetry.addData("Smoothed Offset (Deg)", Math.toDegrees(smoothedAngularOffset));
        telemetry.addData("Final Target (Deg)", Math.toDegrees(finalTargetTurretAngle));
    }

    // --- BUFFER MANAGEMENT METHODS ---

    private void updateHistoryBuffer(long timestampMs, Pose pose, double turretAngle) {
        historyBuffer.put(timestampMs, new RobotState(pose, turretAngle));
        if (historyBuffer.size() > MAX_HISTORY_SIZE) {
            historyBuffer.pollFirstEntry();
        }
    }

    private RobotState getHistoricalState(long targetTimestampMs) {
        if (historyBuffer.isEmpty()) return null;

        Map.Entry<Long, RobotState> floor = historyBuffer.floorEntry(targetTimestampMs);
        Map.Entry<Long, RobotState> ceiling = historyBuffer.ceilingEntry(targetTimestampMs);

        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        // Interpolation
        RobotState floorState = floor.getValue();
        RobotState ceilingState = ceiling.getValue();

        double proportion = (double) (targetTimestampMs - floor.getKey()) / (ceiling.getKey() - floor.getKey());

        // Interpolate Pose
        double interpX = floorState.pose.getX() + (ceilingState.pose.getX() - floorState.pose.getX()) * proportion;
        double interpY = floorState.pose.getY() + (ceilingState.pose.getY() - floorState.pose.getY()) * proportion;

        double headingDiff = angleWrap(ceilingState.pose.getHeading() - floorState.pose.getHeading());
        double interpHeading = angleWrap(floorState.pose.getHeading() + (headingDiff * proportion));

        Pose interpPose = new Pose(interpX, interpY, interpHeading);

        // Interpolate Turret Angle
        double turretDiff = angleWrap(ceilingState.turretAngle - floorState.turretAngle);
        double interpTurretAngle = angleWrap(floorState.turretAngle + (turretDiff * proportion));

        return new RobotState(interpPose, interpTurretAngle);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}