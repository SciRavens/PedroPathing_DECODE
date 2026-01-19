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

@Autonomous(name = "New Red Far Auto", group = "Autonomous")
@Configurable
public class NewRedFarAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "RED";

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(88, 7, Math.toRadians(90));
    private final Pose scoringPose = new Pose(84, 20, Math.toRadians(90));

    private final Pose intakePose1 = new Pose(127, 83.5, Math.toRadians(0));
    private final Pose intakePose1Control1 = new Pose(88, 91.5);
    private final Pose intakePose1Control2 = new Pose(70.5, 85.5);

    private final Pose openGatePose = new Pose(131, 65.5, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(98, 58);
    private final Pose shootControlPoint = new Pose(88, 61);

    private final Pose intakePose2 = new Pose(127, 59, Math.toRadians(0));
    private final Pose intakePose2Control1 = new Pose(82, 75);
    private final Pose intakePose2Control2 = new Pose(73, 58);

    private final Pose intakePose3 = new Pose(127, 35, Math.toRadians(0));
    private final Pose intakePose3Control1 = new Pose(88, 42.5);
    private final Pose intakePose3Control2 = new Pose(69, 34);

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
        public PathChain scoreStack1;
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

            scoreStack1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            intakePose1,
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
                if (pathTimer.getElapsedTime() > 1500) {
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack3);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.intake.stopIntake();
//                    follower.followPath(paths.openGate);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
//                    follower.followPath(paths.scoreWithControl);
                    follower.followPath(paths.scoreStack3); //comment this for gate
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
                    follower.followPath(paths.intakeStack1);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreStack1);
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
