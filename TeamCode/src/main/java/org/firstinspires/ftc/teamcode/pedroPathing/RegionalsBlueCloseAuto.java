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

@Autonomous(name = "Regionals Blue Close Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
@Configurable
public class RegionalsBlueCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Timer pathTimer, opmodeTimer;
    private Vision vision;
    private Robot robot;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private String currentAlliance = "BLUE";

    // ---------------- POSES ----------------
    // Mirrored over x = 72: new_x = 144 - old_x

    private final Pose startPose = new Pose(25, 131, Math.toRadians(145));
    private final Pose scoringPose = new Pose(96, 96, Math.toRadians(90));
    private final Pose scoringPose2 = new Pose(84, 84, Math.toRadians(180)); // stopped here
    private final Pose scoringPose3 = new Pose(81, 71.5, Math.toRadians(0));

    private final Pose intakePose1 = new Pose(144, 61.5, Math.toRadians(0));
    private final Pose intakePose1Control1 = new Pose(98.5, 47.5);
    private final Pose intakePose1Control2 = new Pose(88, 64.5);

    private final Pose openGatePose = new Pose(128.5, 71, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(112.5, 65);
    private final Pose shootControlPoint = new Pose(102.5, 67);

    private final Pose intakePose2 = new Pose(131, 87.5, Math.toRadians(180));
    private final Pose intakePose2Control1 = new Pose(91.5, 86);
    private final Pose intakePose2Control2 = new Pose(108.5, 84);

    private final Pose intakePose3 = new Pose(135, 35, Math.toRadians(180));
    private final Pose intakePose3Control1 = new Pose(102, 28);
    private final Pose intakePose3Control2 = new Pose(76, 38);

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

        pathState = 0;

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
        panelsTelemetry.debug("Shooter RPM", robot.shooter.getCurrentRPM());
        panelsTelemetry.debug("current RPM", robot.shooter.currentRPM);
        // panelsTelemetry.debug("Shooter Target RPM", robot.shooter.);
        panelsTelemetry.update(telemetry);
    }

    private boolean pathWait(long timeoutMs) {
        Pose targetPose = follower.getPose();
        double deltaX = Math.abs(follower.getPose().getX() - targetPose.getX());
        double deltaY = Math.abs(follower.getPose().getY() - targetPose.getY());
        double tolerance = 2.0;
        if (pathTimer.getElapsedTime() > timeoutMs || (deltaX < tolerance && deltaY < tolerance)) {
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
        public PathChain gateBack;
        public PathChain gateScore;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25, 131),

                                    new Pose(64.990, 77.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))

                    .build();

            intakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.990, 77.900),
                                    new Pose(63.940, 48.687),
                                    new Pose(68.709, 63.893),
                                    new Pose(0, 58.833)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            scoreWithControl = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(0, 58.833),
                                    new Pose(41.736, 62.467),
                                    new Pose(64.963, 78.127)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            openGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.963, 78.127),
                                    new Pose(17.069, 48.905),
                                    new Pose(8.826, 68.324)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160))

                    .build();

            gateBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.826, 68.324),

                                    new Pose(8.565, 62.890)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(155))

                    .build();

            gateScore = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.565, 54.890),
                                    new Pose(44.599, 55.064),
                                    new Pose(64.900, 77.686)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))

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
                if (pathWait(1500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
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
                if (pathWait(1500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot first stack
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(30);
                    }
                }
                break;
            case 30:
                if (pathWait(2000)) {
                    follower.followPath(paths.gateBack);
                    setPathState(4);
                }
                break;

            case 4:
                if (pathWait(2500) && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(5);
                }
                break;

            case 5:
                if (pathWait(1500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(50);
                    }
                }
                break;
            case 50:
                if (pathWait(2000)) {
                    follower.followPath(paths.gateBack);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathWait(2500) && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathWait(1500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(70);
                    }
                }
                break;
            case 70:
                if (pathWait(2000)) {
                    follower.followPath(paths.gateBack);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathWait(2500) && pathTimer.getElapsedTime()>2000) { // open gate
                    robot.intake.stopIntake();
                    follower.followPath(paths.gateScore);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathWait(1500)) {
                    completed = robot.autonRapidShoot(follower, 2000);
                    if (completed) { // shoot with gate open
                        robot.intake.startIntake();
                        follower.followPath(paths.openGate);
                        setPathState(-1);
                    }
                }
                break;

            default:
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
