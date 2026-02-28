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
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Autonomous(name = "Regionals Red Close Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RegionalsRedCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(111.545, 136.428, Math.toRadians(90));
    private final Pose scoringPose = new Pose(48, 96, Math.toRadians(90));
    private final Pose scoringPose2 = new Pose(60, 84, Math.toRadians(0)); // stopped here
    private final Pose scoringPose3 = new Pose(63, 71.5, Math.toRadians(180));

    private final Pose intakePose1 = new Pose(0, 61.5, Math.toRadians(180));
    private final Pose intakePose1Control1 = new Pose(45.5, 47.5);
    private final Pose intakePose1Control2 = new Pose(56, 64.5);

    private final Pose openGatePose = new Pose(15.5, 71, Math.toRadians(180));
    private final Pose openGateControlPoint = new Pose(31.5, 65);
    private final Pose shootControlPoint = new Pose(41.5, 67);

    private final Pose intakePose2 = new Pose(13, 87.5, Math.toRadians(0));
    private final Pose intakePose2Control1 = new Pose(52.5, 86);
    private final Pose intakePose2Control2 = new Pose(35.5, 84);

    private final Pose intakePose3 = new Pose(9, 35, Math.toRadians(0));
    private final Pose intakePose3Control1 = new Pose(42, 28);
    private final Pose intakePose3Control2 = new Pose(68, 38);

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

//        pathState = 0;

        vision = new Vision(hardwareMap, robot, follower, telemetry);
        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());


        telemetry.addData("Saved Position Heading (deg): ", Math.toDegrees(SavePosition.getSavedPosition().getHeading()));
        telemetry.addData("Current Alliance: ", currentAlliance);
        telemetry.addData("Vision/Pipeline1", robot.current_pipeline_id);
        telemetry.addData("Vision/TargetTag", robot.current_tag_id);
        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
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
        vision.update(robot.getDistanceFromGoal(follower));
        follower.update();
        robot.shooter.update();
        SavePosition.saveCurrentPosition(follower.getPose());


        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter RPM", robot.shooter.getCurrentRPM());
//        panelsTelemetry.debug("Shooter Target RPM", robot.shooter.);
        panelsTelemetry.update(telemetry);
    }

    private boolean pathWait(long timeoutMs) {
        if(!follower.isBusy() || pathTimer.getElapsedTime() > timeoutMs)  {
            return true;
        }
        return false;
    }
    // ---------------- PATHS ----------------

    public static class Paths {
        public PathChain scorePreload;
        public PathChain intakeStack1;
        public PathChain scoreWithControl;
        public PathChain openGate;
        public PathChain gateScore;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.545, 136.428),

                                    new Pose(95.722, 96.398)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.722, 96.398),
                                    new Pose(100.239, 52.237),
                                    new Pose(78.356, 58.819),
                                    new Pose(129.799, 59.522)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            scoreWithControl = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.799, 59.522),
                                    new Pose(95.309, 60.015),
                                    new Pose(95.669, 96.615)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.669, 96.615),
                                    new Pose(110.122, 35.130),
                                    new Pose(133.639, 60.408)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(35))

                    .build();

            gateScore = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.639, 60.408),
                                    new Pose(110.256, 35.192),
                                    new Pose(95.890, 96.418)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))

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
                if (pathWait(2000)) {
                    completed = robot.autonShoot(follower, 3000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.intakeStack1);
                        setPathState(2);
                    }
                }

                break;
            case 2:
                if (pathWait(2500)) { // intake first stack
                    robot.intake.stopIntake();
                    follower.followPath(paths.scoreWithControl);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathWait(2000)) {
                    completed = robot.autonShoot(follower,3000);
                    if (completed) { // shoot first stack
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(4);
                    }
                }
                break;
                case 4:
                if (pathWait(2500)) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(2000)) {
                    completed = robot.autonShoot(follower,3000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (pathWait(2500)) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(-1);
                }
                break;
            case 7:
                if (pathWait(2000)) {
                    completed = robot.autonShoot(follower, 3000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(8);
                    }
                }



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


