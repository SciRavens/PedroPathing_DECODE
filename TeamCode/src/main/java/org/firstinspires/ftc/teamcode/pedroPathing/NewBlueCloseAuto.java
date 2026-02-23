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
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Blue Close Gate Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class NewBlueCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Turret turret;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "BLUE";
    private boolean turretOn = true;

    // ---------------- POSES ----------------

    private final Pose startPose = new Pose(26.7, 132.7, Math.toRadians(145));
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

//        vision = new Vision(hardwareMap, robot, follower, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        vision = new Vision(hardwareMap, robot, follower, telemetry);
        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());
        telemetry.addData("Pose X: ", follower.getPose().getX());
        telemetry.addData("Pose Y: ", follower.getPose().getY());


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
        if(turretOn) {
            vision.update(robot.getDistanceFromGoal(follower));
        }
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
                                    new Pose(26.7, 132.7),

                                    new Pose(62.000, 82.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))

                    .build();

            IntakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.000, 82.000),
                                    new Pose(58.400, 54.500),
                                    new Pose(66.400, 60.000),
                                    new Pose(0.500, 60.440)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(0.50, 60.440),
                                    new Pose(32.339, 56.611),
                                    new Pose(27.426, 63.260),
                                    new Pose(9.518, 68.192)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ScoreStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.518, 80.192),
                                    new Pose(58, 50),
                                    new Pose(61.900, 81.100)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            IntakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.900, 75.100),
                                    new Pose(56.4,90),
                                    new Pose(8, 88.738)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ScoreStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8, 88.738),

                                    new Pose(62.000, 81.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            IntakeStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.000, 39.600),
                                    new Pose(0.50, 39.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ScoreStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(0.50, 41.400),

                                    new Pose(61.900, 82.300)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            FinishPath = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.900, 82.300),

                                    new Pose(25.000, 77.000)
                            )
                    ).setConstantHeadingInterpolation(180)

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
                robot.shooter.startAutoCloseBlueShoot();
                follower.followPath(paths.ScorePreload);
                turretOn = true;
                setPathState(1);
                break;
            case 1:
                if (pathWait(1000)) {
                    completed = robot.autonRapidShoot(follower, 1500);
                    if (completed) { // shoot preload
                        turretOn = false;
                        robot.intake.startIntake();
                        follower.followPath(paths.IntakeStack1);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (pathWait(2750)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.OpenGate);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathWait(2000)) {
                    follower.followPath(paths.ScoreStack1);
                    turretOn = true;
                    setPathState(5);
                }
                break;
            case 5:
                if (pathWait(3000)) {
                    completed = robot.autonRapidShoot(follower, 1500);
                    if (completed) { // shoot preload
                        turretOn = false;
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
                    turretOn = true;
                    setPathState(8);
                }
                break;

            case 8:
                if (pathWait(2250)) {
                    completed = robot.autonRapidShoot(follower, 1500);
                    if (completed) { // shoot preload
                        turretOn = false;
                        robot.intake.startIntake();
                        follower.followPath(paths.IntakeStack3);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (pathWait(3000)) {
                    robot.gate.gateClose();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ScoreStack3);
                    turretOn = true;
                    setPathState(11);
                }
                break;
            case 11:
                if (pathWait(2500)) {
                    completed = robot.autonRapidShoot(follower, 1500);
                    if (completed) { // shoot preload
                        turretOn = false;
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


