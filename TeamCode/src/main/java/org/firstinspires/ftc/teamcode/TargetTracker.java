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
    private PIDFController turretPIDF;

    public TargetTracker(HardwareMap hardwareMap, Robot robot, Follower follower, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.pipelineSwitch(robot.current_pipeline_id);
        this.limelight.start();
        this.follower = follower;
        this.turret = robot.turret;
        this.robot = robot;
        this.telemetry = telemetry;
        this.turretPIDF = new PIDFController(new PIDFCoefficients(TURRET_KP, 0, TURRET_KD, 0));
        // Change the direction
        this.robot.turret.turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        Pose currentPose = follower.getPose();
        double robotHeading = currentPose.getHeading();
        double currentTurretAngle = turret.getTurretAngleRadians();
        // 1. Calculate the camera's global angle on the field (in degrees for Limelight)
        double cameraGlobalHeading = Math.toDegrees(angleWrap(robotHeading + currentTurretAngle));

        // 2. Send it to the Limelight BEFORE grabbing the result
        limelight.updateRobotOrientation(cameraGlobalHeading);

        // 1. If Limelight has a good target, update the offsets
        if (isRobotStationary()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getTa() > 0.5) {
                double currentTimestamp = result.getTimestamp();
                if (currentTimestamp > lastResultTimestamp) {
                    Pose3D botpose = result.getBotpose_MT2();
                    if (botpose != null) {
                        double limelightX = botpose.getPosition().x * 39.3701;
                        double limelightY = botpose.getPosition().y * 39.3701;

                        offsetX = limelightX - currentPose.getX();
                        offsetY = limelightY - currentPose.getY();
                    }
                    lastResultTimestamp = currentTimestamp;
                }
            }
        }

        // 2. Calculate the TRUE position by combining Odometry + Offset
        double trueX = currentPose.getX() + offsetX;
        double trueY = currentPose.getY() + offsetY;

        // 3. Aim the turret using the TRUE position
        double deltaX = robot.current_goal_x - trueX;
        double deltaY = robot.current_goal_y - trueY;
        double fieldTargetAngle = Math.atan2(deltaY, deltaX);

        // 4. Calculate the target angle for the turret relative to the chassis
        double targetTurretAngle = angleWrap(fieldTargetAngle - robotHeading);

        // 5. Enforce your 180-degree physical limits on the target
        double maxLimit = Math.toRadians(180);
        targetTurretAngle = Math.max(-maxLimit, Math.min(maxLimit, targetTurretAngle));


        // Calculate the exact, shortest-path distance
        double error = angleWrap(targetTurretAngle - currentTurretAngle);

        // Send the TRUE error to the PID!
        turretPIDF.updateError(error);
        double power = turretPIDF.run();

        // ----------------------------------------
        // SAFETY CLAMPS (Now correctly comparing Radians to Radians)
        // ----------------------------------------
        if (currentTurretAngle >= MAX_LEFT_LIMIT && power > 0) {
            power = 0;
        }
        else if (currentTurretAngle <= MAX_RIGHT_LIMIT && power < 0) {
            power = 0;
        }

        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        turret.setTurretPower(power);

        // Helpful Telemetry
        telemetry.addData("Turret Target (Deg)", Math.toDegrees(targetTurretAngle));
        telemetry.addData("Turret Current (Deg)", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Turret Power", power);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public boolean isRobotStationary() {
        // 1. Get the translational velocity Vector from Pedro Pathing
        Vector velocity = follower.getVelocity();

        // 2. Use the built-in magnitude method to get total speed in inches per second
        double translationalSpeed = velocity.getMagnitude();

        // 3. Define our "Stationary" threshold
        double maxTranslationalSpeed = 1.0; // inches per second

        // 4. (Optional but recommended) Check rotational speed too.
        // Depending on your version of Pedro Pathing, you can usually grab
        // the angular velocity directly from the Follower or PoseUpdater.
        // Example: double rotationalSpeed = Math.abs(follower.getHeadingVelocity());
        // If you don't have this method easily accessible, checking translation is often good enough!

        // 5. Return true ONLY if the speed is below the threshold
        return (translationalSpeed < maxTranslationalSpeed);
    }


}