package org.firstinspires.ftc.teamcode.disabled;

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

//import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;
@Disabled
@Autonomous(name = "New Blue Close Auto Test", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class BlueCloseAutoTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
//    private Vision vision;
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

//        vision = new Vision(hardwareMap, robot, follower, telemetry);
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
//        vision.update(robot.getDistanceFromGoal(follower));
        follower.update();
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
                                    new Pose(32.562, 136.910),

                                    new Pose(47.619, 95.448)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                    .build();

            initialStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.619, 95.448),

                                    new Pose(47.445, 64.441)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.445, 64.441),

                                    new Pose(8.829, 64.077)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.829, 64.077),
                                    new Pose(29.737, 69.025),
                                    new Pose(15.836, 71.084)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.829, 64.077),
                                    new Pose(53.860, 71.067),
                                    new Pose(48.060, 95.746)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            initialStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.060, 95.746),

                                    new Pose(46.967, 89.197)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.967, 89.197),

                                    new Pose(18.344, 89.224)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.344, 89.224),

                                    new Pose(47.569, 95.452)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            initialStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.569, 95.452),

                                    new Pose(46.204, 40.338)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            intakeStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.204, 40.338),

                                    new Pose(13.037, 40.890)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            scoreStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.037, 40.890),

                                    new Pose(23.836, 69.084)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

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
                        follower.followPath(paths.initialStack1);
                        setPathState(2);
                    }
                }

                break;
            case 2:
                if (pathWait(500)){
                    //robot.shooter.startAutoCloseBlueShoot();
                    follower.followPath(paths.intakeStack1);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathWait(3000)) {
                    follower.followPath(paths.scoreStack1);
                    setPathState(5);
                }
                break;

            case 5:
                if (pathWait(2000)) {
                    completed = robot.autonShoot(follower, 3000);
                    if (completed) {
                        follower.followPath(paths.initialStack2);
                       // robot.shooter.startAutoCloseBlueShoot();
                        robot.intake.startIntake();
                        setPathState(6);
                    }
                }

                break;

            case 6:
                if (pathWait(500)){
                    follower.followPath(paths.intakeStack2);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathWait(2000)) {
                    follower.followPath(paths.scoreStack2);
                    setPathState(8);
                }
                break;

            case 8:
                if (pathWait(2000)){
                    completed = robot.autonShoot(follower, 3000);
                    if (completed) {
                        //robot.shooter.startAutoCloseBlueShoot();
                        follower.followPath(paths.initialStack3);
                        robot.intake.startIntake();
                        setPathState(9);
                    }
                }
                break;


            case 9:
                if (pathWait(3000)){
                    follower.followPath(paths.intakeStack3);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathWait(3000)) {
                    follower.followPath(paths.scoreStack3);
                    setPathState(-1);
                }
                break;
            case 11:
                if (pathWait(2000)){
                    completed = robot.autonShoot(follower, 4000);
                    if (completed) { //shoot third stack
                        //robot.shooter.startAutoCloseBlueShoot();
                        follower.followPath(paths.initialStack2);
                        setPathState(12);
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


