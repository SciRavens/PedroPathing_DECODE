package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.util.Timer;

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

    // --- NEW TOGGLE VARIABLES ---
    private boolean flywheelRunning = false;
    private boolean lastGamepad2A = false;

    @Override
    public void init() {
        timer = new Timer();
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
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.startTeleopDrive();
        follower.setMaxPower(1.0);
    }

    private boolean is_Intaking() { return gamepad1.right_trigger > 0.1; }
    private boolean is_OpenGate() { return gamepad1.left_bumper; }
//    private boolean is_CloseGate() { return gamepad2.left_bumper; }
//    private boolean is_SmartShooting() { return gamepad2.triangle; }
//    private boolean is_ReverseIntaking() { return gamepad2.right_bumper; }
    private boolean is_Shooting() { return gamepad1.right_bumper;}

    @Override
    public void loop() {
        // Drivetrain Logic
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;
        double powerScale = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(yInput * powerScale, xInput * powerScale, turnInput * powerScale, true);
        follower.update();

        currentPose = follower.getPose();
        double distance = robot.getDistanceFromGoal(follower);

        if (targetTracking_enabled && !robot.turret.isSearching()) {
            ttracker.update();
        }

        // --- FLYWHEEL TOGGLE LOGIC ---
        if (gamepad1.b && !lastGamepad2A) {
            flywheelRunning = !flywheelRunning;
        }
        lastGamepad2A = gamepad1.b;

//        if (is_SmartShooting()) {
//            smartShooting = true;
//            flywheelRunning = true; // Ensure flywheel is on if we start smart shooting
//        }

        if (smartShooting) {
            boolean completed = robot.autonShoot(follower, 2000);
            if (completed) {
                smartShooting = false;
                isFarShootingMode = false;
            }
        }

        // Shooter Mode Logic
        if (gamepad2.b) {
            isFarShootingMode = true;
        } else if (gamepad2.x) {
            isFarShootingMode = false;
        }

        // Apply Motor Power based on Toggle State
        if (flywheelRunning) {
            if (isFarShootingMode) {
                if (distance > 143) { // 143
                    robot.shooter.startShooterbyDistance(distance);
//                    robot.shooter.startAutonFarShoot();
                }
            } else {
                if (distance < 123) { // < 123
                    robot.shooter.startShooterbyDistance(distance);
//                    robot.shooter.startAutonFarShoot();
                } else {
                    robot.shooter.startClosePassiveShoot();
                }
            }
        } else {
            robot.shooter.stopShoot();
        }

        // Intake/Gate/Turret Logic
        if (is_Shooting()) {
            robot.intake.startIntake();
            robot.gate.gateOpen();
        } else if (is_Intaking()) {
            robot.intake.startIntake();
            robot.gate.gateClose();
//        } else if (is_ReverseIntaking()) {
//            robot.intake.startReverseIntake();
//            robot.gate.gateClose();
        } else {
            robot.intake.stopIntake();
            // Allow manual gate control if not shooting or intaking
            if (is_OpenGate()) robot.gate.gateOpen();
            else robot.gate.gateClose();
        }

        if (gamepad2.dpad_up) targetTracking_enabled = true;
        else if (gamepad2.dpad_down) targetTracking_enabled = false;

        if (is_OpenGate()) robot.gate.gateOpen();
//        else if (is_CloseGate()) robot.gate.gateClose();

        if (gamepad1.dpad_right) {
            robot.turret.setTurretPower(-0.5);
        } else if (gamepad1.dpad_left) {
            robot.turret.setTurretPower(0.5);
        } else {
            robot.turret.setTurretPower(0); // Zero power when released
        }

//        if (gamepad1.x) robot.turret.startCentering(Turret.SearchDirection.LEFT);
//        else if (gamepad1.b) robot.turret.startCentering(Turret.SearchDirection.RIGHT);

        SavePosition.saveCurrentPosition(currentPose);
        robot.shooter.update(robot);

        // Telemetry
        telemetry.addData("Flywheel Active", flywheelRunning);
        telemetry.addData("Goal Distance: ", distance);
        telemetry.addData("Gate: ", robot.gate.isGateClosed() ? "CLOSED" : "OPEN");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
    }
}