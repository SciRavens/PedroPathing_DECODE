package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Red Far Auto", group = "Competition", preselectTeleOp="RobotTeleop")
public class RedFar12 extends OpMode {
    private Robot robot;
    private Follower follower;
    private Vision vision;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private TargetTracker ttracker;
    private Path goToFirstPattern, shootStack1, goToSecondPattern, shootStack2, goToThirdPattern, shootStack3, endingAuton, intakeThirdPattern, intakeSecondPattern, intakeFirstPattern;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(0));
    private final Pose firstPattern = new Pose(96.7, 83.6, Math.toRadians(0));
    private final Pose getFirstPattern = new Pose(134, 83.6, Math.toRadians(0));
    private final Pose secondPattern = new Pose(96,58,Math.toRadians(0));
    private final Pose getSecondPattern = new Pose(141, 58, Math.toRadians(0));
    private final Pose thirdPattern = new Pose(96, 36, Math.toRadians(0));
    private final Pose getThirdPattern = new Pose(141, 36, Math.toRadians(0));
    private final Pose shootingPose = new Pose(89, 12, Math.toRadians(0));
    private final Pose finalPose = new Pose(120, 10, Math.toRadians(0));



    @Override
    public void init() {
        Robot.currentAlliance = "RED";
        // Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
        robot = new Robot(hardwareMap,telemetry);
        follower = Constants.createFollower(hardwareMap);
//        ttracker = new TargetTracker(hardwareMap,robot,follower,telemetry);
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
        vision.update(robot.getDistanceFromGoal(follower));
        follower.update();
        robot.shooter.update(robot);
        autonomousPathUpdate();
        SavePosition.saveCurrentPosition(follower.getPose());
//        ttracker.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("path timer", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Gate state", robot.gate.isGateClosed());
        telemetry.addData("Current RPM", robot.shooter.getCurrentRPM());
        telemetry.update();
    }

    private boolean pathWait(long timeoutMs) {
        if(!follower.isBusy() || pathTimer.getElapsedTime() > timeoutMs)  {
            return true;
        }
        return false;
    }

    public void autonomousPathUpdate() {
        boolean completed = false;

        switch (pathState) {
            case 0: // start shooter
                completed = robot.autonShoot(follower, 2900);
                if (completed) {
                    follower.followPath(goToThirdPattern, true);
                    robot.intake.startIntake();
                    setPathState(1);
                }
                break;
//            case 2:
//                if (pathTimer.getElapsedTime() > 1500){ // after finishing the shoot, close gate and go to first stack
//                    robot.gate.gateClose();
//                    follower.followPath(goToThirdPattern, true);
//                    setPathState(3);
//                }
//                break;
            case 1:
                if (pathWait(800)){ // get first stack
                    follower.followPath(intakeThirdPattern, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathWait(1200))  { // after getting first stack, stop intake and go to shoot
                    follower.followPath(shootStack1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathWait(1800)) {
                    //robot.gate.gateOpen();
                    setPathState(4);
                }
                break;
            case 4:
                completed = robot.autonShoot(follower, 2700);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(goToSecondPattern);
                    robot.intake.startIntake();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(1300)){ // get second stack
                    follower.followPath(intakeSecondPattern, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathWait(1300))  { // go to shoot first stack
                    follower.followPath(shootStack2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathWait(1900)) {
                    //robot.gate.gateOpen();
                    setPathState(8);
                }
                break;
            case 8:
                completed = robot.autonShoot(follower, 2700);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(goToFirstPattern);
                    robot.intake.startIntake();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathWait(1800)){ // get third stack
                    follower.followPath(intakeFirstPattern, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathWait(1300))  { // after intaking third stack, go to shoot
                    follower.followPath(shootStack3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathWait(2200)) {
                    //robot.gate.gateOpen();
                    setPathState(12);
                }
                break;
            case 12:
                completed = robot.autonShoot(follower, 2500);
                if (completed) {// shoot third stack
                    robot.gate.gateClose();
                    follower.followPath(endingAuton);
                    robot.intake.stopIntake();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathWait(1000)) {// get away from tape to not lose ranking point
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

