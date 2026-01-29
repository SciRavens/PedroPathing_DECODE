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
public class BlueFar12 extends OpMode {
    private Robot robot;
    private Follower follower;
    private Vision vision;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path goToFirstPattern, shootStack1, goToSecondPattern, shootStack2, goToThirdPattern, shootStack3, endingAuton, intakeThirdPattern, intakeSecondPattern, intakeFirstPattern;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(180));
    private final Pose firstPattern = new Pose(47.3, 83.6, Math.toRadians(180));
    private final Pose getFirstPattern = new Pose(14, 83.6, Math.toRadians(180));
    private final Pose secondPattern = new Pose(48,58,Math.toRadians(180));
    private final Pose getSecondPattern = new Pose(0.5, 58, Math.toRadians(180));
    private final Pose thirdPattern = new Pose(48, 36, Math.toRadians(180));
    private final Pose getThirdPattern = new Pose(0.5, 36, Math.toRadians(180));
    private final Pose shootingPose = new Pose(55, 12, Math.toRadians(180));
    private final Pose finalPose = new Pose(24, 10, Math.toRadians(180));



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
                completed = robot.autonShoot(follower, 3000);
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
                if (pathWait(25000)){ // get first stack
                    follower.followPath(intakeThirdPattern, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathWait(2000))  { // after getting first stack, stop intake and go to shoot
                    robot.intake.stopIntake();
                    follower.followPath(shootStack1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathWait(2500)) {
                    //robot.gate.gateOpen();
                    setPathState(4);
                }
                break;
            case 4:
                completed = robot.autonShoot(follower, 4000);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(goToSecondPattern);
                    robot.intake.startIntake();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(2500)){ // get second stack
                    follower.followPath(intakeSecondPattern, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathWait(2500))  { // go to shoot first stack
                    robot.intake.stopIntake();
                    follower.followPath(shootStack2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathWait(3000)) {
                    //robot.gate.gateOpen();
                    setPathState(12);
                }
                break;
            case 8:
                completed = robot.autonShoot(follower, 4000);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(goToFirstPattern);
                    robot.intake.startIntake();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathWait(3000)){ // get third stack
                    follower.followPath(intakeFirstPattern, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathWait(4000))  { // after intaking third stack, go to shoot
                    robot.intake.stopIntake();
                    follower.followPath(shootStack3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathWait(2000)) {
                    //robot.gate.gateOpen();
                    setPathState(12);
                }
                break;
            case 12:
                completed = robot.autonShoot(follower, 4000);
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

