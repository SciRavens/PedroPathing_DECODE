//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.SavePosition;
//import org.firstinspires.ftc.teamcode.Vision;
//import org.firstinspires.ftc.teamcode.Robot;
//import com.pedropathing.util.Timer;
//
//@Autonomous(name = "New Red Close Auto", group = "Autonomous", preselectTeleOp = "RobotTeleop")
//@Configurable
//public class NewRedCloseAuto extends OpMode {
//
//    private TelemetryManager panelsTelemetry;
//    private Timer pathTimer, opmodeTimer;
//    private Vision vision;
//    private Robot robot;
//    public Follower follower;
//    private int pathState;
//    private Paths paths;
//    private String currentAlliance = "RED";
//
//    // ---------------- POSES ----------------
//
//    private final Pose startPose = new Pose(111, 136.5, Math.toRadians(90));
//    private final Pose scoringPose = new Pose(96, 96, Math.toRadians(90));
//    private final Pose scoringPose2 = new Pose(87.5, 83, Math.toRadians(0));
//    private final Pose scoringPose3 = new Pose(86, 73, Math.toRadians(0));
//
//    private final Pose intakePose1 = new Pose(135, 59.5, Math.toRadians(0));
//    private final Pose intakePose1Control1 = new Pose(100, 51);
//    private final Pose intakePose1Control2 = new Pose(80, 60);
//
//    private final Pose openGatePose = new Pose(131, 73, Math.toRadians(0));
//    private final Pose openGateControlPoint = new Pose(98, 58);
//    private final Pose shootControlPoint = new Pose(88, 61);
//
//    private final Pose intakePose2 = new Pose(131, 86, Math.toRadians(0));
//    private final Pose intakePose2Control1 = new Pose(95, 75);
//    private final Pose intakePose2Control2 = new Pose(91, 86);
//
//    private final Pose intakePose3 = new Pose(128, 35.5, Math.toRadians(0));
//    private final Pose intakePose3Control1 = new Pose(103, 32);
//    private final Pose intakePose3Control2 = new Pose(81, 35);
//
//    // ---------------- INIT ----------------
//
//    @Override
//    public void init() {
//        Robot.currentAlliance = "RED";
//        robot = new Robot(hardwareMap, telemetry);
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        paths = new Paths(follower);
//
////        pathState = 0;
//
//        vision = new Vision(hardwareMap, robot, follower, telemetry);
//        telemetry.addData("Saved Position X: ", SavePosition.getSavedPosition().getX());
//        telemetry.addData("Saved Position Y: ", SavePosition.getSavedPosition().getY());
//        telemetry.addData("Saved Position Heading (deg): ", Math.toDegrees(SavePosition.getSavedPosition().getHeading()));
//        telemetry.addData("Current Alliance: ", currentAlliance);
//        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
//        telemetry.update();
//    }
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        follower.setMaxPower(1.0);
//        pathState = 0;
//    }
//
//    // ---------------- LOOP ----------------
//
//    @Override
//    public void loop() {
//        vision.update();
//        follower.update();
//        SavePosition.saveCurrentPosition(follower.getPose());
//
//
//        pathState = autonomousPathUpdate();
//
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//    private boolean pathWait(long timeoutMs) {
//        if(!follower.isBusy() || pathTimer.getElapsedTime() > timeoutMs)  {
//            return true;
//        }
//        return false;
//    }
//
//    // ---------------- PATHS ----------------
//
//    public class Paths {
//
//        public PathChain scorePreload;
//        public PathChain intakeStack1;
//        public PathChain openGate;
//        public PathChain scoreWithControl;
//        public PathChain intakeStack2;
//        public PathChain scoreStack2;
//        public PathChain intakeStack3;
//        public PathChain scoreStack3;
//        public PathChain scoreStack1;
//
//        public Paths(Follower follower) {
//
//            scorePreload = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(111.438, 136.910),
//                                    new Pose(96.381, 95.448)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
//                    .build();
//
//            initialStack1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(96.381, 95.448),
//                                    new Pose(96.555, 64.441)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                    .build();
//
//            intakeStack1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(96.555, 64.441),
//                                    new Pose(138.171, 64.077)
//                            )
//                    ).setTangentHeadingInterpolation()
//                    .build();
//
//            openGate = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(135.171, 64.077),
//                                    new Pose(116.263, 67.025),
//                                    new Pose(129.164, 69.084)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            scoreStack1 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(135.171, 64.077),
//                                    new Pose(95.140, 71.067),
//                                    new Pose(95.940, 95.746)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                    .build();
//
//            initialStack2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(95.940, 95.746),
//                                    new Pose(97.033, 89.197)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            intakeStack2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(97.033, 89.197),
//                                    new Pose(125.656, 89.224)
//                            )
//                    ).setTangentHeadingInterpolation()
//                    .build();
//
//            scoreStack2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(125.656, 89.224),
//                                    new Pose(96.431, 95.452)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                    .build();
//
//            initialStack3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(96.431, 95.452),
//                                    new Pose(97.796, 40.338)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            intakeStack3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(97.796, 40.338),
//                                    new Pose(138.963, 40.890)
//                            )
//                    ).setTangentHeadingInterpolation()
//                    .build();
//
//            scoreStack3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(130.963, 40.890),
//                                    new Pose(113.164, 69.084)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                    .build();
//
//        }
//    }
//
//    // ---------------- STATE MACHINE ----------------
//
//    public int autonomousPathUpdate() {
//        boolean completed = false;
//        switch (pathState) {
//
//            case 0:
//                follower.followPath(paths.shootPreload);
//                robot.shooter.startAutoCloseBlueShoot();
//
//                setPathState(1);
//                break;
//            case 1:
//                if (pathWait(2000)) {
//                    completed = robot.autonShoot(follower, 3000);
//                    if (completed) { // shoot preload
//                        robot.intake.startIntake();
//                        follower.followPath(paths.initialStack1);
//                        setPathState(2);
//                    }
//                }
//
//                break;
//            case 2:
//                if (pathWait(500)){
//                    //robot.shooter.startAutoCloseBlueShoot();
//                    follower.followPath(paths.intakeStack1);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (pathWait(3000)) {
//                    follower.followPath(paths.scoreStack1);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (pathWait(2000)) {
//                    completed = robot.autonShoot(follower, 3000);
//                    if (completed) {
//                        follower.followPath(paths.initialStack2);
//                        // robot.shooter.startAutoCloseBlueShoot();
//                        robot.intake.startIntake();
//                        setPathState(6);
//                    }
//                }
//
//                break;
//
//            case 6:
//                if (pathWait(500)){
//                    follower.followPath(paths.intakeStack2);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (pathWait(2000)) {
//                    follower.followPath(paths.scoreStack2);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (pathWait(2000)){
//                    completed = robot.autonShoot(follower, 3000);
//                    if (completed) {
//                        //robot.shooter.startAutoCloseBlueShoot();
//                        follower.followPath(paths.initialStack3);
//                        robot.intake.startIntake();
//                        setPathState(9);
//                    }
//                }
//                break;
//
//
//            case 9:
//                if (pathWait(3000)){
//                    follower.followPath(paths.intakeStack3);
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                if (pathWait(3000)) {
//                    follower.followPath(paths.scoreStack3);
//                    setPathState(-1);
//                }
//                break;
//            case 11:
//                if (pathWait(2000)){
//                    completed = robot.autonShoot(follower, 4000);
//                    if (completed) { //shoot third stack
//                        //robot.shooter.startAutoCloseBlueShoot();
//                        follower.followPath(paths.initialStack2);
//                        setPathState(12);
//                    }
//                }
//                break;
//
//            default:
//                break;
//        }
//        return pathState;
//    }
//    public void setPathState (int pState){
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
//
//
