package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;

/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control (CRServo).
 *
 * Author:
 * Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 * 3.2, Added Slew Rate Limiting
 */
@TeleOp(name = "RobotTeleop", group = "Competition")
public class RobotTeleop extends OpMode {
    private Timer timer;
    int counter;
    private Follower follower;
    private Robot robot;
    public Gate gate;

    private Vision vision;
    private TargetTracker ttracker;
    private static final double DEAD_ZONE = 0.1;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private Pose currentPose = new Pose(0,0,0);
    private boolean targetTracking_enabled = true;
    private boolean smartShooting = false;
    private boolean isFarShootingMode = false;

    // --- SLEW RATE LIMITERS ---
    // The number represents the maximum allowed change per second.
    // 2.5 means it takes 0.4 seconds to go from 0 to 1.0 power.
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(2.5);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.5);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(3.0);

    @Override
    public void init() {
        timer = new Timer();
        // Get the Saved position but only if the saved position valid.
        // Holding the circle button will set the default position
        Pose savedPose = new Pose(startPose.getX(), startPose.getY(), startPose.getHeading());
        telemetry.addData("Circle:", gamepad1.circle);
        if (!gamepad1.circle && (SavePosition.getSavedPosition().getX() > 0 && SavePosition.getSavedPosition().getY() > 0)) {
            savedPose = SavePosition.getSavedPosition().copy();
            telemetry.addLine("Using Saved Position");
        } else {
            telemetry.addLine("Using Default Position");
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(savedPose);
        follower.startTeleopDrive();

        if(gamepad1.y) {
            Robot.currentAlliance = "BLUE";
        } else if (gamepad1.x) {
            Robot.currentAlliance = "RED";
        }
        robot = new Robot(hardwareMap, telemetry);
        ttracker = new TargetTracker(hardwareMap, robot, follower, telemetry);
        telemetry.addData("Current Alliance: ", Robot.currentAlliance);
        telemetry.addLine()
                .addData("Current Position X: ", follower.getPose().getX())
                .addData("Y: ", follower.getPose().getY())
                .addData(" Heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine("RobotTeleop Initialization COMPLETE");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.startTeleopDrive();
        follower.setMaxPower(1.0);
    }

    private boolean is_Intaking() { return gamepad2.right_trigger > 0.1; }
    private boolean is_OpenGate() { return gamepad2.left_trigger > 0.1; }
    private boolean is_CloseGate() { return gamepad2.left_bumper; }
    private boolean is_ShootingClose() {return gamepad2.circle;}
    private boolean is_ShootingMiddle() {return gamepad2.triangle;}
    private boolean is_ShootingFar() {return gamepad2.square;}
    private boolean is_FlywheelOff() {return gamepad2.a;}
    private boolean is_FlyWheelOn() {return gamepad2.b;}
    private boolean is_SmartShooting() {return gamepad2.triangle;}
    private boolean is_ReverseIntaking() {return gamepad2.right_bumper;}
    private boolean cancel_smartShooting() { return gamepad2.x; }

    @Override
    public void loop() {
        // 1. Get raw inputs from gamepad
        double rawX = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double rawY = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double rawTurn = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        // 2. Apply Slew Rate Limiter using current time
        double currentTime = timer.getElapsedTimeSeconds();
        double xInput = strafeLimiter.calculate(rawX, currentTime);
        double yInput = forwardLimiter.calculate(rawY, currentTime);
        double turnInput = turnLimiter.calculate(rawTurn, currentTime);

        // 3. Apply Power Scaling
        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(
                yInput * powerScale,  // forward/backward
                xInput * powerScale,  // strafe
                turnInput * powerScale, // rotation (negated)
                true                     // robot-centric
        );
        follower.update();

        currentPose = follower.getPose();
        double distance  = robot.getDistanceFromGoal(follower);
        if (targetTracking_enabled && !robot.turret.isSearching()) {
            ttracker.update();
        }

        if (is_SmartShooting()) {
            smartShooting = true;
        }

        if (smartShooting) {
            boolean completed = robot.autonShoot(follower, 2000);
            if (completed) {
                smartShooting = false;
                isFarShootingMode = false;
            }
        }

        if (gamepad2.b) {
            isFarShootingMode = true; // Intent: I'm going to shoot from far
        } else if (gamepad2.x) {
            isFarShootingMode = false; // Intent: Back to normal close shooting
        }

        if (isFarShootingMode) {
            if (distance > 143) {
                robot.shooter.startShooterbyDistance(distance);
            }
        } else {
            if (distance < 120) {
                robot.shooter.startShooterbyDistance(distance);
            } else {
                robot.shooter.startClosePassiveShoot();
            }
        }

        if (is_Intaking()) {
            robot.intake.startIntake();
        } else if (is_ReverseIntaking()) {
            robot.intake.startReverseIntake();
        } else if (!smartShooting) {
            robot.intake.stopIntake();
        }

        if (gamepad2.dpad_up) {
            targetTracking_enabled = true;
        } else if (gamepad2.dpad_down) {
            targetTracking_enabled = false;
        }

        if (is_OpenGate()) {
            robot.gate.gateOpen();
        } else if (is_CloseGate()) {
            robot.gate.gateClose();
        }

        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.turret.setTurretPower(-0.5);
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.turret.setTurretPower(0.5);
        }

        if (gamepad1.x) {
            robot.turret.startCentering(Turret.SearchDirection.LEFT);
        } else if (gamepad1.b) {
            robot.turret.startCentering(Turret.SearchDirection.RIGHT);
        }

        SavePosition.saveCurrentPosition(currentPose);
        robot.shooter.update(robot);

        telemetry.addData("Current Alliance: ", Robot.currentAlliance);
        telemetry.addLine()
                .addData("Goal X: ", robot.current_goal_x)
                .addData(" Y: ", robot.current_goal_y);
        telemetry.addData("Tracking Target", targetTracking_enabled);
        telemetry.addLine()
                .addData("Pose X: ", currentPose.getX())
                .addData("Y: ", currentPose.getY())
                .addData(" Heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine()
                .addData("RPM Target: ", robot.shooter.getRpmbyDistance(distance))
                .addData("Actual: ", robot.shooter.getCurrentRPM());
        telemetry.addData("Goal Distance: ", distance);
        telemetry.addData("Gate: ", robot.gate.isGateClosed() ? "CLOSED" : "OPEN");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
    }

    /**
     * Helper Class: Slew Rate Limiter
     * Limits the rate of change of a value to prevent sudden, jerky movements.
     */
    private static class SlewRateLimiter {
        private final double rateLimit;
        private double prevVal = 0;
        private double prevTime = 0;

        public SlewRateLimiter(double rateLimit) {
            this.rateLimit = rateLimit;
        }

        public double calculate(double targetVal, double currentTime) {
            double dt = currentTime - prevTime;

            // Handle initialization gap or massive lag spikes
            if (dt < 0 || dt > 1.0) {
                dt = 0;
            }

            double error = targetVal - prevVal;
            double maxChange = rateLimit * dt;

            if (Math.abs(error) > maxChange) {
                prevVal += Math.signum(error) * maxChange;
            } else {
                prevVal = targetVal;
            }

            prevTime = currentTime;
            return prevVal;
        }
    }
}