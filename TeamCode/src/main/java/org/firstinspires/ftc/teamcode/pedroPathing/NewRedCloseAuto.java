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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Autonomous(name = "New Red Close Auto", group = "Autonomous")
@Configurable
public class NewRedCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(111, 136.5, Math.toRadians(90));
    private final Pose scoringPose = new Pose(96, 96, Math.toRadians(90));

    private final Pose intakePose1 = new Pose(128, 59.5, Math.toRadians(0));
    private final Pose intakePose1Control1 = new Pose(100, 51);
    private final Pose intakePose1Control2 = new Pose(80, 60);

    private final Pose openGatePose = new Pose(131, 65.5, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(98, 58);
    private final Pose shootControlPoint = new Pose(88, 61);

    private final Pose intakePose2 = new Pose(131, 83, Math.toRadians(0));
    private final Pose intakePose2Control1 = new Pose(95, 75);
    private final Pose intakePose2Control2 = new Pose(91, 86);

    private final Pose intakePose3 = new Pose(128, 35.5, Math.toRadians(0));
    private final Pose intakePose3Control1 = new Pose(103, 32);
    private final Pose intakePose3Control2 = new Pose(81, 35);

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

//        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ---------------- PATHS ----------------

    public class Paths {

        public PathChain shootPreload;
        public PathChain intakeStack1;
        public PathChain openGate;
        public PathChain scoreWithControl;
        public PathChain intakeStack2;
        public PathChain scoreStack2;
        public PathChain intakeStack3;
        public PathChain scoreStack3;

        public Paths(Follower follower) {

            shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoringPose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            intakeStack1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            scoringPose,
                            intakePose1Control1,
                            intakePose1Control2,
                            intakePose1
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intakePose1,
                            openGateControlPoint,
                            openGatePose
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scoreWithControl = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            openGatePose,
                            shootControlPoint,
                            scoringPose
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            intakeStack2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            scoringPose,
                            intakePose2Control1,
                            intakePose2Control2,
                            intakePose2
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            scoreStack2 = follower.pathBuilder()
                    .addPath(new BezierLine(intakePose2, scoringPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            intakeStack3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            scoringPose,
                            intakePose3Control1,
                            intakePose3Control2,
                            intakePose3
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            scoreStack3 = follower.pathBuilder()
                    .addPath(new BezierLine(intakePose3, scoringPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                robot.shooter.startAutoCloseShoot();
                follower.followPath(paths.shootPreload);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()){
                    robot.intake.startIntakeOnly();
                    robot.gate.gateOpen();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.intake.stopIntake();
                    follower.followPath(paths.openGate);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreWithControl);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    robot.intake.startIntakeOnly();
                    robot.gate.gateOpen();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack2);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreStack2);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    robot.intake.startIntakeOnly();
                    robot.gate.gateOpen();
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack3);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreStack3);
                    robot.intake.startIntakeOnly();
                    robot.gate.gateOpen();
                    pathState = -1;
                }
                break;

            default:
                break;
        }

        return pathState;
    }
}
