package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class TargetTracker {
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private Telemetry telemetry;

    // --- TUNING VALUES ---
    // Start with these. If it oscillates, lower KP. If it's sluggish, increase KP.
    private static final double TURRET_KP = 0.045; //0.045;
    private static final double TURRET_KD = 0.003; // 0.0035;

    // Minimum power to overcome friction (Kickstart)
    private static final double TURRET_K_STATIC = 0;// 0.15;

    // Chassis Feedforward: Counters robot rotation.
    // Set to 0.0 initially. Increase to ~0.01 - 0.05 to make turret stay still when robot spins.
    private static final double TURRET_HEADING_FF = 0;

    // --- SAFETY LIMITS ---
    private static final double MAX_LEFT_LIMIT = Math.toRadians(180.0);
    private static final double MAX_RIGHT_LIMIT = Math.toRadians(-180.0);
    private static final double TURRET_MAX_POWER = 0.3;

    // Distance threshold for applying offset (in FEET)
    // Offset is only applied when tag is farther than this distance
    private static final double FAR_ZONE_THRESHOLD_INCHES = 96.0;

    // Optional: Scale the offset based on how far off-center the tag is (viewing angle)
    // When tag is centered (0 degrees), no offset is applied
    // When tag is at an angle, offset scales proportionally
    // Set to 0.0 to use fixed offset, or ~0.1-0.3 for angle-proportional offset
    private static final double AIM_OFFSET_SCALE_FACTOR = 0.0;

    // State Variables
    private double lastResultTimestamp = 0;
    private double offsetX = 0;
    private double offsetY = 0;

    public TargetTracker(HardwareMap hardwareMap, Robot robot, Follower follower, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(robot.current_pipeline_id);
        this.limelight.start();
        this.follower = follower;
        this.turret = robot.turret;
        this.robot = robot;
        this.telemetry = telemetry;
        // Change the direction
        this.robot.turret.turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        Pose currentPose = follower.getPose();
        double robotHeading = currentPose.getHeading();
        double angularOffset = 0;


        // 1. Get the CURRENT angle of the turret from your hardware encoder
        double currentTurretAngle = turret.getTurretAngleRadians();

        // -----------------------------------------------------------------
        // STEP 1: ODOMETRY "BEST GUESS" TARGETING
        // -----------------------------------------------------------------
        double deltaX = robot.current_goal_x - currentPose.getX();
        double deltaY = robot.current_goal_y - currentPose.getY();
        double fieldTargetAngle = Math.atan2(deltaY, deltaX);

        // Odometry's pure mathematical guess for where the turret should point
        double odomTargetTurretAngle = angleWrap(fieldTargetAngle - robotHeading);

        // -----------------------------------------------------------------
        // STEP 2: LIMELIGHT VISION CORRECTION (ANGULAR OFFSET)
        // -----------------------------------------------------------------
        // Only trust vision when the robot is stable to avoid motion blur/latency
        if (isRobotStationary()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                boolean foundExpectedTag = false;
                double txDegrees = 0;

                // Search all visible tags for the specific goal ID
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == robot.current_tag_id) {
                        foundExpectedTag = true;
                        txDegrees = tag.getTargetXDegrees();
                        break; // Found it! Stop searching.
                    }
                }

                // If we found the correct tag, calculate the drift offset
                if (foundExpectedTag) {
                    double txRadians = Math.toRadians(txDegrees);

                    // TRUE ANGLE: Limelight tx is positive when target is to the right.
                    // Because right is negative radians, we subtract tx.
                    double trueTargetTurretAngle = currentTurretAngle - txRadians;

                    // The difference between Reality (Vision) and Guess (Odometry)
                    angularOffset = angleWrap(trueTargetTurretAngle - odomTargetTurretAngle);

                    // Helpful Vision Telemetry
                    telemetry.addData("Limelight Locked ID:", robot.current_tag_id);
                    telemetry.addData("Limelight tx (Deg)", txDegrees);
                    telemetry.addData("Calculated Offset (Deg)", Math.toDegrees(angularOffset));
                }
            }
        }

        // -----------------------------------------------------------------
        // STEP 3: APPLY OFFSET AND ENFORCE LIMITS
        // -----------------------------------------------------------------
        // Combine Odometry's continuous math with the saved Limelight correction
        double finalTargetTurretAngle = angleWrap(odomTargetTurretAngle + angularOffset);

        // Clamp to prevent breaking wires (Ensure MAX_RIGHT and MAX_LEFT are in radians!)
        finalTargetTurretAngle = Math.max(MAX_RIGHT_LIMIT, Math.min(MAX_LEFT_LIMIT, finalTargetTurretAngle));

        // -----------------------------------------------------------------
        // STEP 4: THE PID FEEDBACK LOOP
        // -----------------------------------------------------------------
        // Calculate the exact, shortest-path distance to the final target
        double error = angleWrap(finalTargetTurretAngle - currentTurretAngle);

        // Feed the true shortest-path error to the PID controller
        turret.turretPIDF.updateError(error);
        double power = turret.turretPIDF.run();

        // -----------------------------------------------------------------
        // STEP 5: HARDWARE SAFETY CLAMPS & MOTOR COMMAND
        // -----------------------------------------------------------------
        // If we are at the physical limits, DO NOT allow power to push further!
        if (currentTurretAngle >= MAX_LEFT_LIMIT && power > 0) {
            power = 0;
        }
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && power < 0) {
            power = 0;
        }

        // Cap the maximum speed of the turret
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));

        // Send power to the motor
        turret.setTurretPower(power);

        // Final System Telemetry
        telemetry.addData("Odom Target (Deg)", Math.toDegrees(odomTargetTurretAngle));
        telemetry.addData("Final Target (Deg)", Math.toDegrees(finalTargetTurretAngle));
        telemetry.addData("Current Turret (Deg)", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Turret Power", power);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public boolean isRobotStationary() {
        return true;
//        // 1. Get the translational velocity Vector from Pedro Pathing
//        Vector velocity = follower.getVelocity();
//
//        // 2. Use the built-in magnitude method to get total speed in inches per second
//        double translationalSpeed = velocity.getMagnitude();
//
//        // 3. Define our "Stationary" threshold
//        double maxTranslationalSpeed = 1.0; // inches per second
//
//        // 4. (Optional but recommended) Check rotational speed too.
//        // Depending on your version of Pedro Pathing, you can usually grab
//        // the angular velocity directly from the Follower or PoseUpdater.
//        // Example: double rotationalSpeed = Math.abs(follower.getHeadingVelocity());
//        // If you don't have this method easily accessible, checking translation is often good enough!
//
//        // 5. Return true ONLY if the speed is below the threshold
//        return (translationalSpeed < maxTranslationalSpeed);
    }


}