//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name = "Two Wheel Shooter RPM Test", group = "Examples")
//public class TwoMotorShooterRPMTest extends OpMode {
//    public int Motor1_currentRPM = 500;
//    public int Motor2_currentRPM = 500;
//    public DcMotorEx shooterMotor1;
//    public DcMotorEx shooterMotor2;
//    public Servo shooterLight;
//    public Robot robot;
//    private Timer buttontimer;
//
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap, telemetry);
//        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
//        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
//        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
//        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
//        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        buttontimer = new Timer();
//        buttontimer.resetTimer();
//    }
//
//    @Override
//    public void start() {
//        buttontimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad2.dpad_up && buttontimer.getElapsedTime() > 500) {
//            Motor1_currentRPM += 25;
//            shooterMotor1.setVelocity(Motor1_currentRPM);
//            buttontimer.resetTimer();
//        } else if (gamepad2.dpad_down && buttontimer.getElapsedTime() > 500) {
//            Motor1_currentRPM-= 25;
//            shooterMotor1.setVelocity(Motor1_currentRPM);
//            buttontimer.resetTimer();
//        } else if (gamepad2.dpad_right && buttontimer.getElapsedTime() > 500) {
//            Motor2_currentRPM += 25;
//            shooterMotor2.setVelocity(Motor2_currentRPM);
//            buttontimer.resetTimer();
//        } else if (gamepad2.dpad_left && buttontimer.getElapsedTime() > 500) {
//            Motor2_currentRPM -= 25;
//            shooterMotor2.setVelocity(Motor2_currentRPM);
//            buttontimer.resetTimer();
//        }
//
//        if (gamepad2.b) {
//            shooterMotor1.setVelocity(robot.shooter.shooterCloseRPM);
//            Motor1_currentRPM = robot.shooter.shooterCloseRPM;
//        } else if (gamepad2.a) {
//            shooterMotor2.setVelocity(0);
//            shooterMotor1.setVelocity(0);
//        } else if (gamepad2.y) {
//            shooterMotor1.setVelocity(robot.shooter.shooterMidRPM);
//            Motor1_currentRPM = robot.shooter.shooterMidRPM;
//        } else if (gamepad2.x) {
//            shooterMotor1.setVelocity(robot.shooter.shooterFarRPM);
//            Motor1_currentRPM = robot.shooter.shooterFarRPM;
//        }
//
////        if (currentRPM > 0 && shooterMotor.getVelocity() >= currentRPM) {
////            shooter.shooterLight.setPosition(0.5);
////        } else {
////            shooterLight.setPosition(0.3);
////        }
//
//        if (gamepad2.right_trigger > 0.1) {
//            robot.intake.startIntakeOnly();
//        } else {
//            robot.intake.stopIntake();
//        }
//
//        telemetry.addData("Motor1 Target RPM", Motor1_currentRPM);
//        telemetry.addData("Motor1 Current Velocity", shooterMotor1.getVelocity());
//        telemetry.addData("Motor1 Motor Direction: ", shooterMotor1.getDirection());
//        telemetry.addData("Motor2 Target RPM", Motor2_currentRPM);
//        telemetry.addData("Motor2 Current Velocity", shooterMotor2.getVelocity());
//        telemetry.addData("Motor2 Motor Direction: ", shooterMotor2.getDirection());
//        telemetry.update();
//    }
//}
//
