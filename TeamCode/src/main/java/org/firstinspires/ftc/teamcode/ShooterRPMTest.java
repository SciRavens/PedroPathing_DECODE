package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter RPM Test", group = "Examples")
public class ShooterRPMTest extends OpMode {
    public int currentRPM = 500;

    public DcMotorEx shooterMotor;
    public Servo shooterLight;

    public Robot robot;
    private Timer buttontimer;



    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        shooterMotor.resetDeviceConfigurationForOpMode();
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                350 ,   // P
                0,       // I
                0,       // D
                20   // F
        );
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
            shooterMotor.setVelocity(currentRPM);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_down && buttontimer.getElapsedTime() > 500) {
            currentRPM -= 25;
            shooterMotor.setVelocity(currentRPM);
            buttontimer.resetTimer();
        }

        if (gamepad2.b) {
            shooterMotor.setVelocity(robot.shooter.shooterCloseRPM);
            currentRPM = robot.shooter.shooterCloseRPM;
        } else if (gamepad2.a) {
            shooterMotor.setVelocity(0);
        } else if (gamepad2.y) {
            shooterMotor.setVelocity(robot.shooter.shooterMidRPM);
            currentRPM = robot.shooter.shooterMidRPM;

        } else if (gamepad2.x) {
            shooterMotor.setVelocity(robot.shooter.shooterFarRPM);
            currentRPM = robot.shooter.shooterFarRPM;
        }

//        if (currentRPM > 0 && shooterMotor.getVelocity() >= currentRPM) {
//            shooter.shooterLight.setPosition(0.5);
//        } else {
//            shooterLight.setPosition(0.3);
//        }

        if (gamepad2.right_trigger > 0.1) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }

        robot.shooter.shooterLightUpdate();
        telemetry.addData("Target RPM", currentRPM);
        telemetry.addData("Current Velocity", shooterMotor.getVelocity());
        telemetry.addData("Motor Direction: ", shooterMotor.getDirection());
        telemetry.update();
    }
}

