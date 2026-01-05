package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Red Far Auto", group = "Competition", preselectTeleOp="RobotTeleop")
public class RedFar12 extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path goToFirstPattern, shootStack1, goToSecondPattern, shootStack2, goToThirdPattern, shootStack3, getFirstPattern, getSecondPattern, getThirdPattern, endingAuton;
    private final Pose startPose = new Pose(83.7, 7.06, Math.toRadians(0));
    private final Pose firstPattern = new Pose(134.35, 83.3, Math.toRadians(0));
    private final Pose controlPoint5 = new Pose(84.44, 104);
    private final Pose controlPoint6 = new Pose(53, 79.47);
    private final Pose secondPattern = new Pose(139, 59.3, Math.toRadians(0));
    private final Pose controlPoint3 = new Pose(84.4, 67.06);
    private final Pose controlPoint4 = new Pose(52.79, 59.88);
    private final Pose thirdPattern = new Pose(98, 36, Math.toRadians(0));
    private final Pose controlPoint1 = new Pose(84.44, 43.06);
    private final Pose controlPoint2 = new Pose(62.67,34);
    private final Pose shootingPose = new Pose(98, 8, Math.toRadians(0));
    private final Pose finalPose = new Pose(120, 10, Math.toRadians(0));



    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
        robot = new Robot(hardwareMap,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        goToThirdPattern = new Path(new BezierCurve(startPose, controlPoint1, controlPoint2, thirdPattern));
        goToThirdPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        shootStack1 = new Path(new BezierLine(thirdPattern, shootingPose));
        shootStack1.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToSecondPattern = new Path(new BezierCurve(shootingPose, controlPoint3, controlPoint4, secondPattern));
        goToSecondPattern.setLinearHeadingInterpolation(shootingPose.getHeading(), secondPattern.getHeading());

        shootStack2 = new Path(new BezierLine(secondPattern, shootingPose));
        shootStack2.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToFirstPattern = new Path(new BezierCurve(shootingPose, controlPoint5, controlPoint6, firstPattern));
        goToFirstPattern.setLinearHeadingInterpolation(shootingPose.getHeading(), thirdPattern.getHeading());

        shootStack3 = new Path(new BezierLine(firstPattern, shootingPose));
        shootStack3.setConstantHeadingInterpolation(shootingPose.getHeading());

        endingAuton = new Path(new BezierLine(shootingPose, finalPose));
        endingAuton.setConstantHeadingInterpolation(shootingPose.getHeading());
//

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setMaxPower(1.0);
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooter.startAutonFarShoot();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.gate.gateOpen();
                    robot.intake.startIntakeOnly();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1500){ //decrease if necessary
                    robot.gate.gateClose();
                    robot.intake.stopIntake();
                    follower.followPath(goToThirdPattern, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(getThirdPattern, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {
                    follower.followPath(shootStack1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 1000) {
                    robot.gate.gateOpen();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.intake.startIntakeOnly();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 1500){
                    robot.gate.gateClose();
                    robot.intake.stopIntake();
                    follower.followPath(goToSecondPattern, true);
                    setPathState(61);
                }
                break;
            case 61:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(getSecondPattern, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2000)  {
                    follower.followPath(shootStack2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 3000) {
                    robot.gate.gateOpen();
                    setPathState(9);
                }
            case 9:
                if (robot.shooter.reachedSpeed()|| pathTimer.getElapsedTime() > 3000) {
                    robot.intake.startIntakeOnly();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 1500){
                    robot.gate.gateOpen();
                    robot.intake.stopIntake();
                    follower.followPath(endingAuton, true);
                    setPathState(-1);
                }
                break;
            case 101:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(getFirstPattern, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 4000)  {
                    follower.followPath(shootStack3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 4000) {
                    robot.gate.gateOpen();
                    setPathState(13);
                }
                break;
            case 13:
                if (robot.shooter.reachedSpeed() || pathTimer.getElapsedTime() > 3000) {
                    robot.intake.startIntakeOnly();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

