package org.firstinspires.ftc.teamcode.disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TargetTracker;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.util.Timer;

@Disabled
@Autonomous(name = "Blue Corner Far", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class BlueCornerFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    private TargetTracker ttracker;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "BLUE";

    // ---------------- POSES ----------------
    // Mirrored over x = 72: new_x = 144 - old_x

    private final Pose startPose = new Pose(55.679, 8.321, Math.toRadians(180));
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
//        vision = vision = new Vision(hardwareMap, robot, follower, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        ttracker =  new TargetTracker(hardwareMap, robot, follower, telemetry);

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
        follower.update();
//        vision.update(robot.getDistanceFromGoal(follower));
        ttracker.update();
        robot.shooter.update(robot);
        SavePosition.saveCurrentPosition(follower.getPose());

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter RPM", robot.shooter.getCurrentRPM());
        panelsTelemetry.debug("current RPM", robot.shooter.currentRPM);
        panelsTelemetry.debug("current alliance", Robot.currentAlliance);
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
        public PathChain InitialStack1;
        public PathChain intakeStack1;
        public PathChain scoreStack1;
        public PathChain intakeCorner;
        public PathChain intakeCorner2;
        public PathChain cornerBack;
        public PathChain cornerBack2;
        public PathChain cornerAgain;
        public PathChain cornerAgain2;
        public PathChain cornerScore;
        public PathChain cornerScore2;

        public Paths(Follower follower) {
            InitialStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.679, 8.321),

                                    new Pose(56.000, 35.518)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 35.518),

                                    new Pose(9.134, 35.468)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.134, 35.468),

                                    new Pose(55.652, 8.284)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            intakeCorner = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.652, 8.284),

                                    new Pose(0.234, 8.341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            intakeCorner2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.652, 8.284),

                                    new Pose(0.234, 40.341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(0, 24.341),

                                    new Pose(15.732, 8.348)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            cornerBack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(0.234, 40.341),

                                    new Pose(15.732, 40.341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerAgain = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.732, 24.348),

                                    new Pose(9.187, 8.181)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerAgain2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.732, 40.341),

                                    new Pose(9.187, 40.341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            cornerScore = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.187, 8.181),

                                    new Pose(55.816, 8.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            cornerScore2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.187, 40.341),

                                    new Pose(55.816, 8.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }

    // ---------------- STATE MACHINE ----------------

    public int autonomousPathUpdate() { // <--- THIS WAS MISSING
        boolean completed = false; // Define this variable for use in cases

        switch (pathState) {
            case 0: // start shooter
                completed = robot.autonShoot(follower, 3000);
                if (completed) {
                    follower.followPath(paths.InitialStack1, true);
                    robot.intake.startIntake();
                    setPathState(1);
                }
                break;

            case 1:
                if (pathWait(800, paths.InitialStack1)){ // Pass the path to pathWait
                    follower.followPath(paths.intakeStack1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(pathWait(1200, paths.intakeStack1))  {
                    robot.intake.startIntake();
                    follower.followPath(paths.scoreStack1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(pathWait(1800, paths.scoreStack1)) {
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2000) {
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        follower.followPath(paths.intakeCorner);
                        robot.intake.startIntake();
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (pathWait(1300, paths.intakeCorner)){
                    follower.followPath(paths.cornerBack);
                    setPathState(6);
                }
                break;

            case 6:
                if(pathWait(1300, paths.cornerBack))  {
                    follower.followPath(paths.cornerAgain);
                    setPathState(7);
                }
                break;

            case 7:
                if(pathWait(1900, paths.cornerAgain)) {
                    follower.followPath(paths.cornerScore);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3000) {
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        // NOTE: Check if 'goToFirstPattern' is defined elsewhere,
                        // otherwise this will cause a "cannot find symbol" error.
                        // follower.followPath(goToFirstPattern);
                        robot.intake.startIntake();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2000){
                    follower.followPath(paths.intakeCorner2);
                    setPathState(10);
                }
                break;

            case 10:
                if(pathWait(1300, paths.intakeCorner2))  {
                    robot.intake.startIntake();
                    follower.followPath(paths.cornerBack2);
                    setPathState(11);
                }
                break;

            case 11:
                if(pathWait(2200, paths.cornerBack2)) {
                    follower.followPath(paths.cornerAgain2);
                    setPathState(12);
                }
                break;

            case 12:
                if (pathWait(2200, paths.cornerAgain2)) {
                    follower.followPath(paths.cornerScore2);
                    setPathState(13);
                }
            case 13:
                if (pathWait(2200, paths.cornerScore2)) {
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        robot.intake.startIntake();
                        setPathState(-1);
                    }
                }


            default:
                break;
        }
        return pathState;
    } // <--- CLOSE THE METHOD HERE

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
