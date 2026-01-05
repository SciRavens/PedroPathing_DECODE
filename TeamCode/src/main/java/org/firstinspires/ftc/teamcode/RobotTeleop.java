package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;


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
    private Timer gateTimer;

    int counter;
    private Timer rapidTimer;

    private Follower follower;
    private Robot robot;
    public Gate gate;


    private Vision vision;
    private TurretTracker turretTracker;
    private static final double DEAD_ZONE = 0.1;
    private static final double TURRET_DEADZONE = 0.3; // Tighter alignment threshold

    private final Pose startPose = new Pose(72, 72, 0);

    private Pose currentPose = new Pose(0,0,0);

    private boolean flywheelOn = false;
    private boolean targetTracking_enabled = true;

    private String currentAlliance = "RED";

    private boolean currentlyShooting = false;


    @Override
    public void init() {

        timer = new Timer();
        gateTimer = new Timer();
        rapidTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        robot = new Robot(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);

        if(gamepad1.y) {
            Robot.current_pipeline_id = Robot.PIPELINE_ID_BLUE;
            Robot.current_tag_id = Robot.BLUE_TARGET_TAG_ID;
            Robot.current_goal_x = Robot.BLUE_GOAL_X;
            Robot.current_goal_y = Robot.BLUE_GOAL_Y;
            currentAlliance = "BLUE";
        }
        else {
            Robot.current_pipeline_id = Robot.PIPELINE_ID_RED;
            Robot.current_tag_id = Robot.RED_TARGET_TAG_ID;
            Robot.current_goal_x = Robot.RED_GOAL_X;
            Robot.current_goal_y = Robot.RED_GOAL_Y;
            currentAlliance = "RED";
        }
//        turretTracker = new TurretTracker(robot);
        vision = new Vision(hardwareMap, robot, telemetry);
        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());
        telemetry.addData("Saved Position Heading (deg): ", Math.toDegrees(SavePosition.getSavedPosition().getHeading()));
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

    private boolean is_Intaking() {
        return gamepad2.right_trigger > 0.1;
    }

    private boolean is_OpeningGate() {
        return gamepad2.left_trigger > 0.1;
    }

    private boolean is_ClosingGate() { return gamepad2.left_bumper; }

    private boolean is_HumanPlayer() {
        return gamepad1.a;
    }

    private boolean is_FlywheelOff() {
        return gamepad2.a;
    }

    private boolean is_DistanceShot() {
        return gamepad2.b;
    }

    private boolean is_ReverseIntaking() {return gamepad2.right_bumper;}

    public double getDistanceFromGoal() {
        return Math.sqrt(Math.pow(Robot.current_goal_x - currentPose.getX(), 2)
                + Math.pow(Robot.current_goal_y - currentPose.getY(), 2));
    }

    public int getTargetShooterRPM(double distance) {
        // y=-0.0000915882x^{4}+0.0393786x^{3}-6.18693x^{2}+426.45244x-9804.42961
        double rpm = -0.0000915882 * Math.pow(distance, 4)
                + 0.0393786 * Math.pow(distance, 3)
                - 6.18693 * Math.pow(distance, 2)
                + 426.45244 * distance - 9804.42961;
        return (int) rpm;
    }


    @Override
    public void loop() {
        currentPose = follower.getPose();
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
        if (targetTracking_enabled) {
            vision.update();
        }

        if (is_DistanceShot()) {
            flywheelOn = true;
        }
        if (is_FlywheelOff()) {
            flywheelOn = false;
        }

        if (flywheelOn) {
            int targetRPM = getTargetShooterRPM(getDistanceFromGoal());
            robot.shooter.startTargetShooterSpeed(targetRPM);

        } else {
            robot.shooter.stopShoot();
        }


        if (is_Intaking()) {
            robot.intake.startIntakeOnly();
        }  else if (is_ReverseIntaking()) {
            robot.intake.startReverseIntake();
        } else {
            robot.intake.stopIntake();
        }


        if (is_OpeningGate() && gate.gateClosed && gateTimer.getElapsedTime() > 300) {
            gate.gateOpen();
            gateTimer.resetTimer();
            telemetry.addData("Gate Input", "Opening");
        }

        if (is_ClosingGate() && !gate.gateClosed && gateTimer.getElapsedTime() > 300) {
            gate.gateClose();
            gateTimer.resetTimer();
            telemetry.addData("Gate Input", "Closing");
        }



        // Turret control (fixed: check gamepad2 on both dpad sides)
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.turret.goRight(); // rotate right
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.turret.goLeft(); // rotate left
        } else {
            robot.turret.stopTurret();
        }

        SavePosition.saveCurrentPosition(currentPose);
        robot.shooter.shooterLightUpdate();
        telemetry.addData("Current Alliance: ", currentAlliance);
        telemetry.addData("Target RPM: ", getTargetShooterRPM(getDistanceFromGoal()));
        telemetry.addData("Current RPM: ", robot.shooter.getCurrentRPM());
        telemetry.addData("Distance From Goal: ", getDistanceFromGoal());
        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        telemetry.addData("Goal X", Robot.current_goal_x);
        telemetry.addData("Goal Y", Robot.current_goal_y);
        telemetry.addData("Pose X", currentPose.getX());
        telemetry.addData("Pose Y", currentPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Gate Closed", gate.gateClosed);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
    }
}