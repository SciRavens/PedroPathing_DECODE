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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Autonomous(name = "Regionals Blue Close Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RegionalsBlueCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "BLUE";

    // ---------------- POSES ----------------
    // Mirrored over x = 72: new_x = 144 - old_x

    private final Pose startPose = new Pose(26.7, 132.7, Math.toRadians(145));
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
        public PathChain scoreWithControl;
        public PathChain openGate;
        public PathChain gateScore;
        public PathChain gateBack;

        public Paths(Follower follower) {
            // Mirrored coordinates: new_x = 144 - old_x
            // Mirrored headings: 0° ↔ 180°, 45° ↔ 135°, 90° stays 90°
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.7, 132.7),
                                    new Pose(65, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(65, 80),
                                    new Pose(43.761, 52.237),
                                    new Pose(65.644, 58.819),
                                    new Pose(1.201, 59.522)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            scoreWithControl = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(1.201, 59.522),
                                    new Pose(48.691, 60.015),
                                    new Pose(65, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(65, 80),
                                    new Pose(33.878, 38.130),
                                    new Pose(6.361, 73.408)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .build();
            gateBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.361, 73.408),
                                    new Pose(6.361, 50.408)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(145))
                    .build();

            gateScore = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(6.361, 50.408),
                                    new Pose(33.744, 45.192),
                                    new Pose(65, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
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
                    follower.followPath(paths.scoreWithControl);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathWait(1800, paths.scoreWithControl)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot first stack
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(30);
                    }
                }
                break;
            case 30:
                if (pathWait(2000, paths.openGate) && pathTimer.getElapsedTime()>1500) {
                    follower.followPath(paths.gateBack);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(5);
                }
                break;

            case 5:
                if (pathWait(1800, paths.gateScore)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(50);
                    }
                }
                break;
            case 50:
                if (pathWait(2000, paths.openGate)) {
                    follower.followPath(paths.gateBack);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathWait(2500, paths.gateBack) && pathTimer.getElapsedTime()>2000) { // open gate
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
                if (pathWait(2500, paths.gateBack) && pathTimer.getElapsedTime()>2000) { // open gate
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
