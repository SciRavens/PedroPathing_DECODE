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
import org.firstinspires.ftc.teamcode.TargetTracker;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.util.Timer;

@Autonomous(name = "Blue Corner Far V2", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class BlueCornerFarV2 extends OpMode {

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

        vision = new Vision(hardwareMap, robot, follower, telemetry);
//        ttracker =  new TargetTracker(hardwareMap, robot, follower, telemetry);

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
        vision.update(robot.getDistanceFromGoal(follower));
//        ttracker.update();
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
        public PathChain initialIntake1;
        public PathChain IntakeStack1;
        public PathChain scoreStack1;
        public PathChain intakeStack2;
        public PathChain scoreStack2;
        public PathChain cornerIntake;
        public PathChain cornerBack;
        public PathChain cornerAgain;
        public PathChain cornerScore;
        public PathChain leave;

        public Paths(Follower follower) {
            initialIntake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 36.000),

                                    new Pose(9.161, 35.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            scoreStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.161, 35.458),

                                    new Pose(50.997, 7.769)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            intakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.997, 7.769),
                                    new Pose(58.370, 45.125),
                                    new Pose(6.788, 42.741),
                                    new Pose(8.356, 53.436),
                                    new Pose(8.803, -1.25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();

            scoreStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.803, 8.140),
                                    new Pose(31.569, 22.508),
                                    new Pose(53.013, 7.906)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            cornerIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.013, 7.906),

                                    new Pose(8.799, 8.007)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.799, 8.007),

                                    new Pose(28.033, 8.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerAgain = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.033, 8.331),

                                    new Pose(8.722, 8.020)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            cornerScore = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.722, 8.020),

                                    new Pose(53.064, 8.030)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.064, 8.030),

                                    new Pose(47.555, 18.214)
                            )
                    ).setTangentHeadingInterpolation()

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
                    follower.followPath(paths.initialIntake1, true);
                    robot.intake.startIntake();
                    setPathState(1);
                }
                break;

            case 1:
                if (pathWait(2800, paths.initialIntake1)){ // Pass the path to pathWait
                    follower.followPath(paths.IntakeStack1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(pathWait(1200, paths.IntakeStack1))  {
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
                        follower.followPath(paths.intakeStack2);
                        robot.intake.startIntake();
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (pathWait(3000, paths.intakeStack2)){
                    follower.followPath(paths.scoreStack2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3000) {
                    completed = robot.autonShoot(follower, 2800);
                    if (completed) {
                        robot.gate.gateClose();
                        follower.setMaxPower(1.0);
                        // NOTE: Check if 'goToFirstPattern' is defined elsewhere,
                        // otherwise this will cause a "cannot find symbol" error.
                        // follower.followPath(goToFirstPattern);
                        robot.intake.startIntake();
                        setPathState(-1);
                    }
                }
                break;


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
