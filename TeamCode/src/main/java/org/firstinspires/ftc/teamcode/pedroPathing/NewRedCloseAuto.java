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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SavePosition;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;
@Autonomous(name = "Red Close Gate Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class NewRedCloseAuto extends OpMode {

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
        public PathChain ScorePreload;
        public PathChain IntakeStack1;
        public PathChain OpenGate;
        public PathChain ScoreStack1;
        public PathChain IntakeStack2;
        public PathChain ScoreStack2;
        public PathChain IntakeStack3;
        public PathChain ScoreStack3;
        public PathChain FinishPath;

        public Paths(Follower follower) {
            ScorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.545, 136.428),

                                    new Pose(82, 82.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            IntakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.000, 82.000),
                                    new Pose(85.600, 54.500),
                                    new Pose(77.600, 60.000),
                                    new Pose(143.000, 59.440)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(142.800, 59.440),
                                    new Pose(111.661, 56.611),
                                    new Pose(116.574, 63.260),
                                    new Pose(134.482, 74.192)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ScoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.482, 74.192),

                                    new Pose(82.100, 82.100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            IntakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.100, 82.100),
                                    new Pose(87.6,90),
                                    new Pose(131.5, 87.738)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ScoreStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.5, 87.738),

                                    new Pose(82, 81.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            IntakeStack3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82, 81.600),
                                    new Pose(92.800, 28.100),
                                    new Pose(76.400, 36.300),
                                    new Pose(142.800, 35.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            ScoreStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(142.800, 35.400),

                                    new Pose(82.100, 82.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            FinishPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.100, 82.300),

                                    new Pose(119.000, 77.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

    private boolean pathWait(long timeoutMs) {
        if(!follower.isBusy() || pathTimer.getElapsedTime() > timeoutMs)  {
            return true;
        }
        return false;
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() {
        boolean completed = false;

        switch (pathState) {

            case 0:
                follower.followPath(paths.ScorePreload);
                setPathState(1);
                break;
            case 1:
                if (pathWait(1000)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.IntakeStack1);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (pathWait(2500)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.OpenGate);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathWait(2000)) {
                    follower.followPath(paths.ScoreStack1);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(3000)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.IntakeStack2);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (pathWait(2000)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ScoreStack2);
                    setPathState(8);
                }
                break;

            case 8:
                if (pathWait(2500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.IntakeStack3);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (pathWait(3500)) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ScoreStack3);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathWait(2500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot preload
                        robot.intake.startIntake();
                        follower.followPath(paths.FinishPath);
                        setPathState(-1);
                    }
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


