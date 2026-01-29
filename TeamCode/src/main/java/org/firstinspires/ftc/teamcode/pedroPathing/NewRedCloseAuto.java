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

@Autonomous(name = "New Red Close Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class NewRedCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(111, 136.5, Math.toRadians(90));
    private final Pose scoringPose = new Pose(96, 96, Math.toRadians(90));
    private final Pose scoringPose2 = new Pose(87.5, 83, Math.toRadians(0));
    private final Pose scoringPose3 = new Pose(86, 73, Math.toRadians(0));

    private final Pose intakePose1 = new Pose(135, 59.5, Math.toRadians(0));
    private final Pose intakePose1Control1 = new Pose(100, 51);
    private final Pose intakePose1Control2 = new Pose(80, 60);

    private final Pose openGatePose = new Pose(131, 73, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(98, 58);
    private final Pose shootControlPoint = new Pose(88, 61);

    private final Pose intakePose2 = new Pose(131, 86, Math.toRadians(0));
    private final Pose intakePose2Control1 = new Pose(95, 75);
    private final Pose intakePose2Control2 = new Pose(91, 86);

    private final Pose intakePose3 = new Pose(128, 35.5, Math.toRadians(0));
    private final Pose intakePose3Control1 = new Pose(103, 32);
    private final Pose intakePose3Control2 = new Pose(81, 35);

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

//        pathState = 0;

        vision = new Vision(hardwareMap, robot, follower, telemetry);
        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());
        telemetry.addData("Saved Position Heading (deg): ", Math.toDegrees(SavePosition.getSavedPosition().getHeading()));
        telemetry.addData("Current Alliance: ", currentAlliance);
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
        vision.update();
        follower.update();
        SavePosition.saveCurrentPosition(follower.getPose());


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
        public PathChain scoreStack1;

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
            scoreStack1 = follower.pathBuilder()
                    .addPath(new BezierLine(openGatePose, scoringPose3))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeStack2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            scoringPose,
                            intakePose2Control1,
                            intakePose2Control2,
                            intakePose2
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            scoreStack2 = follower.pathBuilder()
                    .addPath(new BezierLine(intakePose2, scoringPose2))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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
                robot.shooter.startAutoCloseRedShoot();
                follower.followPath(paths.shootPreload);
                robot.gate.gateOpen();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                    robot.intake.startIntake();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.shooter.startAutoMidRedShoot();
                    robot.intake.stopIntake();
                    follower.followPath(paths.openGate);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreStack1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    robot.intake.startIntake();
                    robot.gate.gateOpen();
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateClose();
//                    robot.intake.stopIntake();
                    follower.followPath(paths.intakeStack2);
                    follower.setMaxPower(1.0);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreStack2);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() || (pathTimer.getElapsedTime() > 4000 && robot.shooter.reachedSpeed())) {
                    robot.intake.startIntake();
                    robot.gate.gateOpen();
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTime() > 3000) {
                    follower.setMaxPower(1.0);
                    robot.gate.gateClose();
                    robot.shooter.startAutoCloseRedShoot();
//                    robot.intake.stopIntake();
                    follower.followPath(paths.intakeStack3);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    robot.gate.gateClose();
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.scoreStack3);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    robot.intake.startIntake();
                    robot.gate.gateOpen();
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


