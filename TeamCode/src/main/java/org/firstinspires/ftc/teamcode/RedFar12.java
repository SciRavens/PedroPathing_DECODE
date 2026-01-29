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
import org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Blue Far Auto", group = "Competition", preselectTeleOp="RobotTeleop")
public class RedFar12 extends OpMode {
    private Robot robot;
    private Follower follower;
    private Vision vision;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path goToFirstPattern, shootStack1, goToSecondPattern, shootStack2, goToThirdPattern, shootStack3, endingAuton, intakeThirdPattern, intakeSecondPattern, intakeFirstPattern;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(180));
    private final Pose firstPattern = new Pose(96.7, 83.6, Math.toRadians(180));
    private final Pose getFirstPattern = new Pose(130, 83.6, Math.toRadians(180));
    private final Pose secondPattern = new Pose(96,60,Math.toRadians(180));
    private final Pose getSecondPattern = new Pose(143.5, 60, Math.toRadians(180));
    private final Pose thirdPattern = new Pose(96, 36, Math.toRadians(180));
    private final Pose getThirdPattern = new Pose(143.5, 36, Math.toRadians(180));
    private final Pose shootingPose = new Pose(89, 12, Math.toRadians(180));
    private final Pose finalPose = new Pose(120, 10, Math.toRadians(180));



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
        vision = new Vision(hardwareMap, robot, follower, telemetry);
    }

    public void buildPaths() {
        goToThirdPattern = new Path(new BezierLine(startPose, thirdPattern));
        goToThirdPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        intakeThirdPattern = new Path(new BezierLine(thirdPattern, getThirdPattern));
        intakeThirdPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        shootStack1 = new Path(new BezierLine(getThirdPattern, shootingPose));
        shootStack1.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToSecondPattern = new Path(new BezierLine(shootingPose, secondPattern));
        goToSecondPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        intakeSecondPattern = new Path(new BezierLine(secondPattern, getSecondPattern));
        intakeSecondPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        shootStack2 = new Path(new BezierLine(getSecondPattern, shootingPose));
        shootStack2.setConstantHeadingInterpolation(shootingPose.getHeading());

        goToFirstPattern = new Path(new BezierLine(shootingPose, firstPattern));
        goToFirstPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        intakeFirstPattern = new Path(new BezierLine(firstPattern, getFirstPattern));
        intakeFirstPattern.setLinearHeadingInterpolation(startPose.getHeading(), thirdPattern.getHeading());

        shootStack3 = new Path(new BezierLine(getFirstPattern, shootingPose));
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
        vision.update();
        follower.update();
        autonomousPathUpdate();
        SavePosition.saveCurrentPosition(follower.getPose());

        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Gate state", robot.gate.isGateClosed());
        telemetry.addData("Current RPM", robot.shooter.getCurrentRPM());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // start shooter
                robot.shooter.startAutonFarShoot();
                robot.gate.gateOpen();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed()) { // open the gate and shoot the preload
                    robot.intake.startIntake();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1500){ // after finishing the shoot, close gate and go to first stack
                    robot.gate.gateClose();
                    follower.followPath(goToThirdPattern, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000){ // get first stack
                    follower.followPath(intakeThirdPattern, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 3000)  { // after getting first stack, stop intake and go to shoot
                    robot.intake.stopIntake();
                    follower.followPath(shootStack1, true);
                    setPathState(41);
                }
                break;
            case 41:
                if(pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateOpen();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()) { //shoot first stack
                    robot.intake.startIntake();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 3000){ // get second stack
                    robot.gate.gateClose();
                    follower.followPath(goToSecondPattern, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000){ // get second stack
                    follower.followPath(intakeSecondPattern, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()|| pathTimer.getElapsedTime() > 3500)  { // go to shoot first stack
                    robot.intake.stopIntake();
                    follower.followPath(shootStack2, true);
                    setPathState(81);
                }
                break;
            case 81:
                if(pathTimer.getElapsedTime() > 1750) {
                    robot.gate.gateOpen();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()) {// shoot second stack
                    robot.intake.startIntake();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 3500){ // after shooting second stack, get third stack
                    robot.gate.gateClose();
                    follower.followPath(goToFirstPattern, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000){ // get third stack
                    follower.followPath(intakeFirstPattern, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() ||  pathTimer.getElapsedTime() > 4000)  { // after intaking third stack, go to shoot
                    robot.intake.stopIntake();
                    follower.followPath(shootStack3, true);
                    setPathState(121);
                }
                break;
            case 121:
                if(pathTimer.getElapsedTime() > 2000) {
                    robot.gate.gateOpen();
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()) {// shoot third stack
                    robot.intake.startIntake();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 2000) {// get away from tape to not lose ranking point
                    robot.intake.stopIntake();
                    robot.gate.gateClose();
                    follower.followPath(endingAuton, true);
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

