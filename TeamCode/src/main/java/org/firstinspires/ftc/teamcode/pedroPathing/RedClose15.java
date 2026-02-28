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

import org.firstinspires.ftc.teamcode.disabled.SavePosition;
import org.firstinspires.ftc.teamcode.TargetTracker;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Autonomous(name = "Red Close 15 Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RedClose15 extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    private TargetTracker ttracker;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    // ---------------- POSES ----------------
    // Mirrored over x = 72: new_x = 144 - old_x

    private final Pose startPose = new Pose(110.903, 135.786, Math.toRadians(90));
//
//    // Points extracted from your working Bezier coordinates
//    private final Pose scorePreloadPose = new Pose(64.990, 77.900, Math.toRadians(135));
//    private final Pose intakeStackPose = new Pose(0, 58.833, Math.toRadians(180));
//    private final Pose scoreWithControlPose = new Pose(64.963, 78.127, Math.toRadians(135));
//    private final Pose openGatePose = new Pose(6.826, 68.324, Math.toRadians(160));
//    private final Pose gateBackPose = new Pose(8.826, 62.890, Math.toRadians(155));
//    private final Pose gateScorePose = new Pose(64.900, 77.686, Math.toRadians(135));
//
//    // Control Points
//    private final Pose intakeStackControl1 = new Pose(63.940, 48.687);
//    private final Pose intakeStackControl2 = new Pose(68.709, 63.893);
//    private final Pose scoreControl1 = new Pose(41.736, 62.467);
//    private final Pose openGateControl1 = new Pose(17.069, 48.905);
//    private final Pose gateScoreControl1 = new Pose(44.599, 55.064);

    // ---------------- INIT ----------------

    @Override
    public void init() {
        Robot.currentAlliance = "BLUE";
        robot = new Robot(hardwareMap, telemetry);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        paths = new Paths(follower);

        pathState = 0;

//        vision = new Vision(hardwareMap, robot, follower, telemetry);
//        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
//        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());
//
//        telemetry.addData("Saved Position Heading (deg): ", Math.toDegrees(SavePosition.getSavedPosition().getHeading()));
//        telemetry.addData("Current Alliance: ", currentAlliance);
//        telemetry.addData("Vision/Pipeline1", robot.current_pipeline_id);
//        telemetry.addData("Vision/TargetTag", robot.current_tag_id);
//        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
//        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setMaxPower(1.0);
        pathState = 0;
    }

    // ---------------- LOOP ----------------

    @Override
    public void loop() {
//        vision.update(robot.getDistanceFromGoal(follower));
        follower.update();
        ttracker.update();
        robot.shooter.update();
        SavePosition.saveCurrentPosition(follower.getPose());

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter RPM", robot.shooter.getCurrentRPM());
        panelsTelemetry.debug("current RPM", robot.shooter.currentRPM);
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

    // ---------------- PATHS ----------------

    public static class Paths {
        public PathChain scorePreload;
        public PathChain intakeStack1;
        public PathChain scoreStack1;
        public PathChain intakeStack2;
        public PathChain scoreStack2;
        public PathChain openGate;
        public PathChain gateBack;
        public PathChain gateScore;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(110.903, 135.786),
                                    new Pose(84.940, 77.753)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.940, 77.753),
                                    new Pose(95.405, 83.097),
                                    new Pose(78.007, 84.264),
                                    new Pose(125.468, 83.886)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.468, 83.886),
                                    new Pose(84.110, 77.796)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.110, 77.796),
                                    new Pose(85.672, 57.533),
                                    new Pose(81.448, 60.139),
                                    new Pose(130.047, 58.943)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.047, 58.943),
                                    new Pose(99.868, 63.446),
                                    new Pose(84.100, 77.729)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.100, 77.729),
                                    new Pose(126.811, 43.590),
                                    new Pose(132.783, 80.080)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            gateBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.783, 80.080),
                                    new Pose(134.652, 60.950)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            gateScore = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.652, 70.950),
                                    new Pose(84.037, 78.027)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(45))
                    .build();
        }
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {
        boolean completed = false;
        switch (pathState) {

            case 0:
                follower.followPath(paths.scorePreload);
                robot.shooter.startAutoCloseBlueShoot();
                setPathState(1);
                break;

            case 1:
                if (pathWait(1500, paths.scorePreload)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.intakeStack1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (pathWait(2500, paths.intakeStack1)) { // intake first stack
                    robot.intake.stopIntake();
                    follower.followPath(paths.scoreStack1);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathWait(1800, paths.scoreStack1)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot first stack
                        robot.intake.startIntake();
                        follower.followPath(paths.intakeStack2);
                        setPathState(30);
                    }
                }
                break;
            case 30:
                if (pathWait(2000, paths.intakeStack2) && pathTimer.getElapsedTime()>1500) {
                    follower.followPath(paths.scoreStack2);
                    setPathState(4);
                }
                break;

            case 4:
                if (pathWait(1800, paths.scoreStack2)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot first stack
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2000) {
                    follower.followPath(paths.gateBack);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathWait(1800, paths.gateScore)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(70);
                    }
                }
                break;
            case 70:
                if (pathWait(2000, paths.openGate)) {
                    follower.followPath(paths.gateBack);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathWait(1800, paths.gateScore)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
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
