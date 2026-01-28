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

@Autonomous(name = "New Red Close Auto Test", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RedCloseAutoTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "BLUE";

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(33.5, 137, Math.toRadians(90));
    private final Pose scoringPose = new Pose(48, 96, Math.toRadians(90));
    private final Pose scoringPose2 = new Pose(60, 84, Math.toRadians(0)); // stopped here
    private final Pose scoringPose3 = new Pose(63, 71.5, Math.toRadians(180));

    private final Pose intakePose1 = new Pose(9, 61.5, Math.toRadians(180));
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
        telemetry.addData("Vision/Pipeline1", Robot.current_pipeline_id);
        telemetry.addData("Vision/TargetTag", Robot.current_tag_id);
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

    public static class Paths {
        public PathChain scorePreload;
        public PathChain initialStack1;
        public PathChain intakeStack1;
        public PathChain openGate;
        public PathChain scoreStack1;
        public PathChain initialStack2;
        public PathChain intakeStack2;
        public PathChain scoreStack2;
        public PathChain initialStack3;
        public PathChain intakeStack3;
        public PathChain scoreStack3;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.706, 136.268),

                                    new Pose(95.813, 95.719)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            initialStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.813, 95.719),

                                    new Pose(96.030, 59.217)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.030, 59.217),

                                    new Pose(134.184, 59.284)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.184, 59.284),
                                    new Pose(117.344, 63.970),
                                    new Pose(128.538, 67.632)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.538, 67.632),
                                    new Pose(101.000, 74.445),
                                    new Pose(95.615, 95.779)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            initialStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.615, 95.779),

                                    new Pose(95.756, 83.706)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.756, 83.706),

                                    new Pose(130.355, 83.799)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.355, 83.799),

                                    new Pose(95.793, 95.602)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            initialStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.793, 95.602),

                                    new Pose(95.826, 35.301)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intakeStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.826, 35.301),

                                    new Pose(135.385, 35.258)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            scoreStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.385, 35.258),

                                    new Pose(95.515, 95.672)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();
        }
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                robot.shooter.startAutoCloseBlueShoot();
                follower.followPath(paths.scorePreload);
                robot.gate.gateOpen();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()){
                    robot.shooter.startAutoMidBlueShoot();
                    robot.intake.feedBalls();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1500) {
                    follower.setMaxPower(0.75);
                    robot.gate.gateClose();
                    follower.followPath(paths.intakeStack1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    robot.intake.stopIntake();
                    follower.followPath(paths.openGate);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>1500) {
                    follower.followPath(paths.scoreStack1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && robot.shooter.reachedSpeed()) {
                    robot.shooter.startAutoCloseBlueShoot();
                    robot.intake.feedBalls();
                    robot.gate.gateOpen();
                    follower.setMaxPower(0.75);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTime() > 2500) {
                    robot.gate.gateClose();
//                    robot.intake.stopIntake();
                    follower.followPath(paths.intakeStack2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 2000) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.scoreStack2);
                    setPathState(8);
                }
                break;

            case 8:
                if ((!follower.isBusy() || pathTimer.getElapsedTime() > 3500) && robot.shooter.reachedSpeed()) {
                    robot.intake.feedBalls();
                    robot.gate.gateOpen();
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTime() > 1500) {
                    follower.setMaxPower(0.75);
                    robot.gate.gateClose();
//                    robot.intake.stopIntake();
                    follower.followPath(paths.intakeStack3);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.scoreStack3);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 5000) {
                    robot.intake.feedBalls();
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


