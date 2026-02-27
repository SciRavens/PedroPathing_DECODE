package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Offset Angle Test", group = "Examples")
public class OffsetAngleTest extends OpMode {
    public double currentOffset = 0.0;
    public Robot robot;
    private Vision vision;
    public double hoodPosition = 0.0;
    //    public Servo hoodservo;
    private Timer buttontimer;
    private Follower follower;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(0));



    @Override
    public void init() {
        Robot.currentAlliance ="RED";
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry);
        vision = new Vision(hardwareMap, robot, follower, telemetry);
        follower.setStartingPose(startPose);


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
        double distance = robot.getDistanceFromGoal(follower);
        if(gamepad2.a) {
            robot.shooter.startShooterbyDistance(distance);
        }
        if(gamepad2.b) {
            robot.intake.startIntake();
        }
        if (gamepad2.dpad_up && buttontimer.getElapsedTime() > 200) {
            currentOffset += 1;
            vision.setOffset(currentOffset);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_down && buttontimer.getElapsedTime() > 200) {
            currentOffset -= 1;
            vision.setOffset(currentOffset);
            buttontimer.resetTimer();
        }

        robot.shooter.update();
        telemetry.addData("Target Offset: ", currentOffset);
        telemetry.addData("Current Velocity Front: ", robot.shooter.shooterMotorFront.getVelocity());
//        telemetry.addData("Current Velocity Back : ", robot.shooter.shooterMotorBack.getVelocity());
//        telemetry.addData("Target Hood position: ", hoodPosition);
//        telemetry.addData("Current Hood position: ", hoodservo.getPosition());
        telemetry.update();
//        follower.update();
        vision.update(distance);
    }
}



