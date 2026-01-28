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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SavePosition;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Autonomous(name = "Red Far Corner", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RedFarCorner extends OpMode {

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
        vision.update();
        follower.update();
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
                    ).setTangentHeadingInterpolation()

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
    private boolean autonShoot(Timer pathTimer, long timeoutMs) {
        if (robot.gate.isGateClosed()) {
            robot.shooter.startAutonFarShoot();
            robot.gate.gateOpen();
        }
        if (robot.shooter.reachedSpeed()) {
            robot.intake.startIntake();
        } else {
            robot.intake.stopIntake();
        }
        if (pathTimer.getElapsedTime() > timeoutMs) {
            robot.gate.gateClose();
            return true;
        }
        return false;
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                boolean completed = autonShoot(pathTimer, 4000);
                if (completed) {
                    follower.followPath(paths.intakeThirdStack, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > 2500)  {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootThirdStack, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2250) {
                    robot.gate.gateOpen();
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.startIntake();
                } else {
                    robot.intake.stopIntake();
                }
                if (pathTimer.getElapsedTime() > 4000) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intakeCornerStack);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(paths.backUpFromIntakingCornerStack);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 250) {
                    follower.followPath(paths.goBackIntoCornerToIntake);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 250) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootingPos);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 500) {
                    robot.gate.gateOpen();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.startIntake();
                } else {
                    robot.intake.stopIntake();
                }
                if (pathTimer.getElapsedTime() > 4000) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intakeCornerStack);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(paths.backUpFromIntakingCornerStack);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 250) {
                    follower.followPath(paths.goBackIntoCornerToIntake);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 500) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.shootingPos);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()) {
                    robot.gate.gateOpen();
                    setPathState(14);
                }
                break;
            case 14:
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.startIntake();
                } else {
                    robot.intake.stopIntake();
                }
                if (pathTimer.getElapsedTime() > 3000) {
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


