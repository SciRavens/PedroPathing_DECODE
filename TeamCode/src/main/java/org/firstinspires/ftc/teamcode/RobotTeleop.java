package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.experimental.LimelightTracker;
import org.firstinspires.ftc.teamcode.experimental.TurretPIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.experimental.RobotTeleopRefactored;

/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control with Limelight auto-tracking.
 *
 * Author:
 *   Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 *   3.2, 11/2025 - Added Limelight vision tracking
 */
@TeleOp(name = "RobotTeleop", group = "Competition")
public class RobotTeleop extends OpMode {
    private Timer timer;
    private Timer rapidTimer;
    private LimelightTracker limelightTracker;
    private TurretPIDController turretController;
    private Follower follower;
    private Robot robot;
    private VisionTest vision;

    private static final double DEAD_ZONE = 0.1;
    private final Pose startPose = new Pose(0, 0, 0);
    private Pose currentPose = new Pose(0, 0, 0);

    private boolean is_RapidFireOn = false;
    private boolean targetTracking_enabled = true;
    private String currentAlliance = "RED";

    @Override
    public void init() {

        timer = new Timer();
        rapidTimer = new Timer();

        // Initialize follower (drive system)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Initialize robot subsystems
        robot = new Robot(hardwareMap, telemetry);

        if(gamepad1.y) {
            Robot.current_pipeline_id = Robot.PIPELINE_ID_BLUE;
            Robot.current_tag_id = Robot.BLUE_TARGET_TAG_ID;
            currentAlliance = "BLUE";
        }
        else{
            Robot.current_pipeline_id = Robot.PIPELINE_ID_RED;
            Robot.current_tag_id = Robot.RED_TARGET_TAG_ID;
            currentAlliance = "RED";
        }
//        turretTracker = new TurretTracker(robot);
        vision = new VisionTest(hardwareMap, robot);
        telemetry.addData("Current Alliance: ", currentAlliance);
        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.startTeleopDrive();
        follower.setMaxPower(1.0);
    }

    // -------------------- GAMEPAD MAPPINGS --------------------

    private boolean is_FarShot() {
        return gamepad2.x;
    }

    private boolean is_Intaking() {
        return gamepad2.left_bumper;
    }

    private boolean is_Shooting() {
        return gamepad2.right_bumper;
    }

    private boolean is_HumanPlayer() {
        return gamepad1.a;
    }

    private boolean is_FlywheelOff() {
        return gamepad2.a;
    }

    private boolean is_ShootingRapidFireCloseRange() {
        return gamepad2.b;
    }

    private boolean is_ShootingRapidFireMidRange() {
        return gamepad2.y;
    }

    private boolean is_TurretLeft() {
        return gamepad2.dpad_left;
    }

    private boolean is_TurretRight() {
        return gamepad2.dpad_right;
    }

    private boolean is_ToggleTracking() {
        return gamepad2.dpad_up;
    }

    // -------------------- MAIN LOOP --------------------

    @Override
    public void loop() {
        // -------------------- DRIVE CONTROL --------------------
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(
                yInput * powerScale,
                xInput * powerScale,
                turnInput * powerScale,
                true
        );
        follower.update();

        // -------------------- VISION TRACKING --------------------
        // Toggle tracking on/off with DPAD_UP
        if (is_ToggleTracking()) {
            targetTracking_enabled = !targetTracking_enabled;
            // Add small delay to prevent multiple toggles
            try { Thread.sleep(200); } catch (InterruptedException e) {}
        }

        // Update vision tracking if enabled
        if (targetTracking_enabled) {
            vision.update();
        }

        // -------------------- MANUAL TURRET OVERRIDE --------------------
        // Manual control overrides auto-tracking (only when both dpad buttons aren't pressed)
        if (is_TurretRight() && !is_TurretLeft()) {
            robot.turret.goRight();
        } else if (is_TurretLeft() && !is_TurretRight()) {
            robot.turret.goLeft();
        } else if (!targetTracking_enabled) {
            // Only stop if tracking is off and no manual input
            robot.turret.stopTurret();
        }
        // If tracking is enabled and no manual input, vision.update() handles it

        // -------------------- SHOOTER CONTROL --------------------
        if (is_FarShot()) {
            robot.shooter.startFarShoot();
        } else if (is_ShootingRapidFireCloseRange() && !is_RapidFireOn) {
            is_RapidFireOn = true;
            robot.shooter.startCloseShoot();
            rapidTimer.resetTimer();
        } else if (is_ShootingRapidFireMidRange() && !is_RapidFireOn) {
            is_RapidFireOn = true;
            robot.shooter.startMidShoot();
            rapidTimer.resetTimer();
        } else if (is_HumanPlayer()) {
            robot.shooter.startHumanIntake();
        } else if (is_FlywheelOff()) {
            robot.shooter.stopShoot();
        }

        // -------------------- INTAKE CONTROL --------------------
        if (is_Intaking()) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }

        // -------------------- RAPID FIRE MODE --------------------
        if (is_RapidFireOn) {
            if (robot.shooter.reachedSpeed()) {
                robot.intake.shootArtifacts();
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
            if (rapidTimer.getElapsedTime() >= 5750) {
                robot.shooter.stopFlyWheel();
                robot.intake.intakeStop();
                robot.intake.stopTransfer();
                is_RapidFireOn = false;
            }
        } else {
            // -------------------- NORMAL SHOOTING --------------------
            if (is_Shooting()) {
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.startTransferOnly();
                    gamepad1.rumble(1000);
                    gamepad2.rumble(1000);
                }
            } else {
                robot.intake.stopTransfer();
            }
        }

        // -------------------- UPDATES --------------------
        currentPose = follower.getPose();
        robot.shooter.shooterLightUpdate();

        telemetry.addData("Current Alliance: ", currentAlliance);
        telemetry.addData("Rapid Fire On: ", is_RapidFireOn);
        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        //telemetry.addData("Turret Power", turretCR.getPower());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Power Scale", powerScale);

        telemetry.addLine("\n=== SHOOTER ===");
        telemetry.addData("Rapid Fire", is_RapidFireOn ? "ON" : "OFF");
//        telemetry.addData("Shooter RPM", "%.0f", robot.shooter.getShooterRPM());

        telemetry.addLine("\n=== TURRET TRACKING ===");
        telemetry.addData("Auto Tracking", targetTracking_enabled ? "ENABLED" : "DISABLED");

        if (targetTracking_enabled) {
            telemetry.addData("Status", vision.getStatus());
            telemetry.addData("Locked", vision.isLocked() ? "YES" : "NO");
            telemetry.addData("Tag ID", vision.getDetectedTagID());
            telemetry.addData("Raw Error", "%.2f°", vision.getRawError());
            telemetry.addData("Filtered Error", "%.2f°", vision.getFilteredError());
            telemetry.addData("Turret Power", "%.3f", vision.getTurretPower());
        } else {
            telemetry.addData("Status", "MANUAL CONTROL");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
        robot.intake.stopTransfer();
        robot.turret.stopTurret();
        vision.stop();
    }
}