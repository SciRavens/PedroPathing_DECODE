package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SavePosition;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;
@Disabled
@Autonomous(name = "Red Far Corner Only", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RedFarCornerV2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(88, 8, Math.toRadians(0));
    private final Pose intakeThirdStackPos = new Pose(134, 83.3, Math.toRadians(0));


    // ---------------- INIT ----------------

    @Override
    public void init() {
        Robot.currentAlliance = "RED";
        robot = new Robot(hardwareMap, telemetry);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        paths = new Paths(follower);

        vision = new Vision(hardwareMap, robot, follower, telemetry);
        telemetry.addData("Current Alliance: ", currentAlliance);
        telemetry.update();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setMaxPower(1);
        pathState = 0;
    }

    // ---------------- LOOP ----------------

    @Override
    public void loop() {
        vision.update(robot.getDistanceFromGoal(follower));
        follower.update();
        robot.shooter.update();
        SavePosition.saveCurrentPosition(follower.getPose());


        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Timer: ", pathTimer.getElapsedTime());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ---------------- PATHS ----------------

    public class Paths {

        public PathChain intakeThirdStack;
        public PathChain shootThirdStack;
        public PathChain intakeCornerStack;
        public PathChain backUpFromIntakingCornerStack;
        public PathChain goBackIntoCornerToIntake;
        public PathChain shootingPos;
        public PathChain leave;

        public Paths(Follower follower) {

            intakeThirdStack = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(96.665, 41.823),
                                    new Pose(109.756, 34.388),
                                    new Pose(120.469, 36.297),
                                    new Pose(132, 35.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
            shootThirdStack = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132, 35.000),
                                    new Pose(94.067, 40.139),
                                    new Pose(88.000, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            intakeCornerStack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),
                                    new Pose(136, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            backUpFromIntakingCornerStack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136, 8.000),

                                    new Pose(110, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            goBackIntoCornerToIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(110, 8.000),

                                    new Pose(136, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            shootingPos = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136, 8.000),

                                    new Pose(88.000, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88, 8.000),

                                    new Pose(100, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }

    private boolean pathWait(long timeoutMs) {
        if(!follower.isBusy() || pathTimer.getElapsedTime() > timeoutMs)  {
            return true;
        }
        return false;
    }

    public int autonomousPathUpdate() {
        boolean completed = false;

        switch (pathState) {
            case 0:
                completed = robot.autonShoot(follower, 5000);
                if (completed) {
                    follower.followPath(paths.intakeCornerStack, true);
                    robot.intake.startIntake();
                    setPathState(2);
                }
                break;

            case 2:
                if (pathWait(2500)) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootingPos, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathWait(2250)) {
                    robot.gate.gateOpen();
                    setPathState(4);
                }
                break;
            case 4:
                completed = robot.autonShoot(follower, 4000);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intakeCornerStack);
                    robot.intake.startIntake();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(2000)) {
                    follower.followPath(paths.backUpFromIntakingCornerStack);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathWait(250)) {
                    follower.followPath(paths.goBackIntoCornerToIntake);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathWait(500)) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootingPos);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathWait(2000)) {
                    robot.gate.gateOpen();
                    setPathState(9);
                }
                break;
            case 9:
                completed = robot.autonShoot(follower, 4000);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intakeCornerStack);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathWait(2000)) {
                    follower.followPath(paths.backUpFromIntakingCornerStack);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathWait(250)) {
                    follower.followPath(paths.goBackIntoCornerToIntake);
                    setPathState(-1);
                }
                break;
            case 12:
                if (pathWait(500)) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootingPos);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathWait(2000)) {
                    robot.gate.gateOpen();
                    setPathState(14);
                }
                break;
            case 14:
                completed = robot.autonShoot(follower, 3000);
                if (completed) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intakeCornerStack);
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
        return pathState;
    }
    public void setPathState (int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}


