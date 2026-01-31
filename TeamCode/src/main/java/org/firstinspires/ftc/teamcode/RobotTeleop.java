package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;


/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control (CRServo).
 *
 * Author:
 *   Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 *   3.1, 10/2025
 */
@TeleOp(name = "RobotTeleop", group = "Competition")
public class RobotTeleop extends OpMode {
    private Timer timer;
    int counter;
    private Follower follower;
    private Robot robot;
    public Gate gate;


    private Vision vision;
    private static final double DEAD_ZONE = 0.1;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private Pose currentPose = new Pose(0,0,0);
    private boolean targetTracking_enabled = true;
    private boolean smartShooting = false;


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
        gate = new Gate(hardwareMap, telemetry);
//        turretTracker = new TurretTracker(robot);
        vision = new Vision(hardwareMap, robot, follower, telemetry);
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
    private boolean is_Intaking() {
        return gamepad2.right_trigger > 0.1;
    }

    private boolean is_OpenGate() {
        return gamepad2.left_trigger > 0.1;
    }

    private boolean is_CloseGate() { return gamepad2.left_bumper; }

    private boolean is_ShootingClose() {return gamepad2.circle;}
    private boolean is_ShootingMiddle() {return gamepad2.triangle;}
    private boolean is_ShootingFar() {return gamepad2.square;}


    private boolean is_FlywheelOff() {return gamepad2.a;}
    private boolean is_FlyWheelOn() {return gamepad2.b;}
    private boolean is_SmartShooting() {return gamepad2.triangle;}

    private boolean is_ReverseIntaking() {return gamepad2.right_bumper;}

    @Override
    public void loop() {
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        // NOTE: rotation is negated to match PedroPathing's TeleOp example (prevents reversed/odd rotation behavior)
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

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
        if (targetTracking_enabled) {
            vision.update(robot.getDistanceFromGoal(follower));
        }

        if (is_SmartShooting()) {
            smartShooting = true;
        }

        if (smartShooting) {
            boolean completed = robot.autonShoot(follower, 4000);
            if (completed) {
                smartShooting = false;
                //robot.shooter.stopShoot();
                robot.shooter.startPassiveShoot(); // Keep the flywheel running at lower speed
            }
        }

        if (is_FlyWheelOn()) {
            robot.shooter.startShooterbyDistance(distance);
        } else if (is_FlywheelOff()) {
            robot.shooter.stopShoot();
        }

        if (is_Intaking()) {
            robot.intake.startIntake();
        } else if (is_ReverseIntaking()) {
            robot.intake.startReverseIntake();
        } else if (!smartShooting) {
            robot.intake.stopIntake();
        }

        if (is_OpenGate()) {
            gate.gateOpen();
        } else if (is_CloseGate()) {
            gate.gateClose();
        }

        // Turret control (fixed: check gamepad2 on both dpad sides)
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.turret.setTurretPower(0.5); // rotate right
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.turret.setTurretPower(-0.5); // rotate left
        }

        SavePosition.saveCurrentPosition(currentPose);
        robot.shooter.shooterLightUpdate();
        telemetry.addData("Current Alliance: ", Robot.currentAlliance);
        telemetry.addLine()
            .addData("Goal X: ", robot.current_goal_x)
            .addData(" Y: ", robot.current_goal_y);
        telemetry.addLine()
            .addData("Pose X: ", currentPose.getX())
            .addData("Y: ", currentPose.getY())
            .addData(" Heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine()
                .addData("RPM Target: ", robot.shooter.getRpmbyDistance(distance))
                .addData("Actual: ", robot.shooter.getCurrentRPM());
        telemetry.addData("Goal Distance: ", distance);
//        telemetry.addData("Drive X", xInput);
//        telemetry.addData("Drive Y", yInput);
//        telemetry.addData("Turn", turnInput);
        telemetry.addData("Gate: ", gate.isGateClosed() ? "CLOSED" : "OPEN");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
    }
}