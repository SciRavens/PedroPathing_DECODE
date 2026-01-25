package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Shooter RPM Test", group = "Examples")
public class ShooterRPMTest extends OpMode {
    public int currentRPM = 500;
    public Robot robot;
    private Vision vision;
    public double hoodPosition = 0.0;
//    public Servo hoodservo;
    private Timer buttontimer;
    private Follower follower;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry);
        vision = new Vision(hardwareMap, robot, follower, telemetry);

//        hoodservo = hardwareMap.get(Servo.class, "shooterHoodServo");
//        hoodservo.setPosition(0.0);
        //shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        buttontimer = new Timer();
        buttontimer.resetTimer();
    }

    @Override
    public void start() {
        buttontimer.resetTimer();
    }

    @Override
    public void loop() {

        if (gamepad2.dpad_up && buttontimer.getElapsedTime() > 500) {
            currentRPM += 25;
            robot.shooter.setRPM(currentRPM);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_down && buttontimer.getElapsedTime() > 500) {
            currentRPM -= 25;
            robot.shooter.setRPM(currentRPM);
            buttontimer.resetTimer();
        }

        if (gamepad2.b) {
            robot.shooter.setRPM(robot.shooter.shooterCloseRPM);
            currentRPM = robot.shooter.shooterCloseRPM;
        } else if (gamepad2.a) {
            robot.shooter.stopShoot();
        } else if (gamepad2.y) {
            robot.shooter.setRPM(robot.shooter.shooterMidRedRPM);
            currentRPM = robot.shooter.shooterMidBlueRPM;

        } else if (gamepad2.x) {
            robot.shooter.setRPM(robot.shooter.shooterFarRPM);
            currentRPM = robot.shooter.shooterFarRPM;
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }
//        if (gamepad2.right_bumper && buttontimer.getElapsedTime() > 500){
//            hoodPosition += 0.01;
//            hoodservo.setPosition(hoodPosition);
//            buttontimer.resetTimer();
//        }
//        if (gamepad2.left_bumper && buttontimer.getElapsedTime() > 500){
//            hoodPosition -= 0.01;
//            hoodservo.setPosition(hoodPosition);
//            buttontimer.resetTimer();
//        }

        robot.shooter.shooterLightUpdate();
        telemetry.addData("Target RPM: ", currentRPM);
        telemetry.addData("Current Velocity Front: ", robot.shooter.shooterMotorFront.getVelocity());
//        telemetry.addData("Current Velocity Back : ", robot.shooter.shooterMotorBack.getVelocity());
//        telemetry.addData("Target Hood position: ", hoodPosition);
//        telemetry.addData("Current Hood position: ", hoodservo.getPosition());
        telemetry.update();
//        follower.update();
        vision.update();
    }
}

