package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;

@Autonomous(name = "Red Corner Far V2", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RedCornerFarV2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    private TargetTracker ttracker;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    private boolean turretOn = true;

    // ---------------- POSES ----------------
    // Mirrored: 144 - 55.679 = 88.321 | 180 - 180 = 0
    private final Pose startPose = new Pose(88.321, 8.321, Math.toRadians(0));

    @Override
    public void init() {
        Robot.currentAlliance = "RED";
        robot = new Robot(hardwareMap, telemetry);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        vision = new Vision(hardwareMap, robot, follower, telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        paths = new Paths(follower);
        pathState = 0;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setMaxPower(1.0);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        robot.shooter.update(robot);
        SavePosition.saveCurrentPosition(follower.getPose());

        pathState = autonomousPathUpdate();

        if (turretOn) {
            vision.update(robot.getDistanceFromGoal(follower));
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("current alliance", Robot.currentAlliance);
        panelsTelemetry.update(telemetry);
    }

    private Pose getTarget(PathChain path) {
        return path.getPath(path.size() - 1).getLastControlPoint();
    }

    private boolean pathWait(long timeoutMs, PathChain path) {
        Pose target = getTarget(path);
        double deltaX = Math.abs(follower.getPose().getX() - target.getX());
        double deltaY = Math.abs(follower.getPose().getY() - target.getY());
        double tolerance = 2.0;
        return (pathTimer.getElapsedTime() > timeoutMs) || (deltaX < tolerance && deltaY < tolerance);
    }

    // ---------------- MIRRORED PATHS ----------------

    public static class Paths {
        public PathChain initialIntake1, IntakeStack1, scoreStack1, intakeStack2, scoreStack2;

        public Paths(Follower follower) {
            initialIntake1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 36.000))
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            IntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(88.000, 36.000), new Pose(134.839, 35.458))
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(134.839, 35.458), new Pose(93.003, 7.769))
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(93.003, 7.769),
                                    new Pose(85.630, 45.125),
                                    new Pose(137.212, 42.741),
                                    new Pose(135.644, 53.436),
                                    new Pose(135.197, 0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.197, 0),
                                    new Pose(112.431, 22.508),
                                    new Pose(90.987, 7.906)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();
        }
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {
        boolean completed = false;
        switch (pathState) {
            case 0:
                turretOn = true;
                completed = robot.autonShoot(follower, 3000);
                if (completed) {
                    turretOn = false;
                    follower.followPath(paths.initialIntake1, true);
                    robot.intake.startIntake();
                    setPathState(1);
                }
                break;

            case 1:
                if (pathWait(2800, paths.initialIntake1)){
                    follower.followPath(paths.IntakeStack1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(pathWait(1200, paths.IntakeStack1))  {
                    robot.intake.startIntake();
                    follower.followPath(paths.scoreStack1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(pathWait(1800, paths.scoreStack1)) {
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    turretOn = true;
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        turretOn = false;
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        follower.followPath(paths.intakeStack2);
                        robot.intake.startIntake();
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (pathWait(2500, paths.intakeStack2)){
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.scoreStack2);
                    turretOn = true;
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3000) {
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        turretOn = false;
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        robot.intake.startIntake();
                        setPathState(-1);
                    }
                }
                break;

            default:
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}