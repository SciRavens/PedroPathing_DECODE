package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import com.pedropathing.util.Timer;

@Autonomous(name = "Blue Corner Far V2", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class BlueCornerFarV2 extends OpMode {

    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private boolean turretOn = true;

    // Mirrored Start Pose: (144 - 88.321) = 55.679 | Heading: 180 - 0 = 180
    private final Pose startPose = new Pose(55.679, 8.321, Math.toRadians(180));

    @Override
    public void init() {
        Robot.currentAlliance = "BLUE"; // Updated for the mirrored side
        robot = new Robot(hardwareMap, telemetry);

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

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // ---------------- MIRRORED PATHS ----------------

    public static class Paths {
        public PathChain initialIntake1, IntakeStack1, scoreStack1, intakeStack2, scoreStack2, leave;

        public Paths(Follower follower) {
            // X reflected: 144 - 88 = 56 | Heading reflected: 180
            initialIntake1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000))
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // X reflected: 144 - 134.839 = 9.161
            IntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(56.000, 36.000), new Pose(9.161, 35.458))
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // X reflected: 144 - 93.003 = 50.997
            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(9.161, 35.458), new Pose(50.997, 7.769))
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Heading -90 remains -90 when mirrored across X=72
            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.997, 7.769),
                                    new Pose(58.370, 45.125),
                                    new Pose(6.788, 42.741),
                                    new Pose(8.356, 53.436),
                                    new Pose(5.803, 0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(5.803, 0),
                                    new Pose(31.569, 22.508),
                                    new Pose(53.013, 7.906)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(53.013, 7.906), new Pose(10.000, 7.906))
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                        follower.followPath(paths.leave);
                        robot.intake.startIntake();
                        setPathState(-1);
                    }
                }
                break;
        }
        return pathState;
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}