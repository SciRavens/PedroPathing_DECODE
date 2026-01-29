package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public static final int BLUE_TARGET_TAG_ID = 20;
    public static final int PIPELINE_ID_BLUE = 8;
    public static final int RED_TARGET_TAG_ID = 24;
    public static final int PIPELINE_ID_RED = 2;
    public static final int RED_GOAL_X = 144;
    public static final int RED_GOAL_Y = 144;
    public static final int BLUE_GOAL_X = 0;
    public static final int BLUE_GOAL_Y = 144;
    public static String currentAlliance = "RED";
    public int current_pipeline_id = PIPELINE_ID_RED;
    public int current_tag_id = RED_TARGET_TAG_ID;
    public int current_goal_x = RED_GOAL_X;
    public int current_goal_y = RED_GOAL_Y;

    private boolean is_autonShootTimerOn = false;
    private Timer shootTimer;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Gate gate;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Robot (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        gate = new Gate(hardwareMap, telemetry);
        shootTimer = new Timer();
        if (currentAlliance.equals("BLUE")) {
            current_pipeline_id = PIPELINE_ID_BLUE;
            current_tag_id = BLUE_TARGET_TAG_ID;
            current_goal_x = BLUE_GOAL_X;
            current_goal_y = BLUE_GOAL_Y;
        } else {
            current_pipeline_id = PIPELINE_ID_RED;
            current_tag_id = RED_TARGET_TAG_ID;
            current_goal_x = RED_GOAL_X;
            current_goal_y = RED_GOAL_Y;
        }
    }

    public double getDistanceFromGoal(Follower follower) {
        double dx = this.current_goal_x - follower.getPose().getX();
        double dy = this.current_goal_y - follower.getPose().getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
//    public boolean autonShootByDistance(Follower follower, long timeoutMs, double distance) {
//        gate.gateOpen();
//        shooter.startShooterbyDistance(distance);
//        if (shooter.reachedSpeed()) {
//            if (!is_autonShootTimerOn) {
//               shootTimer.resetTimer();
//               is_autonShootTimerOn = true;
//            }
//            intake.feedBalls();
//        } else {
//            intake.stopFeeding();
//        }
//        if (is_autonShootTimerOn && shootTimer.getElapsedTime() > timeoutMs) {
//            intake.stopFeeding();
//            gate.gateClose();
//            is_autonShootTimerOn = false;
//            return true;
//        }
//        return false;
//    }


    public boolean autonShootByDistance(Follower follower, long timeoutMs, double distance) {
        gate.gateOpen();
        shooter.startShooterbyDistance(distance);

        if (is_autonShootTimerOn) {
            if (shooter.isSafeToContinueShooting()) {
                intake.feedBalls();
            } else {
                intake.stopFeeding();
            }
        } else if (shooter.reachedSpeed()) {
            shootTimer.resetTimer();
            is_autonShootTimerOn = true;
            intake.feedBalls();
        } else {
            intake.stopFeeding();
        }

        if (is_autonShootTimerOn && shootTimer.getElapsedTime() > timeoutMs) {
            intake.stopFeeding();
            gate.gateClose();
            is_autonShootTimerOn = false;
            return true;
        }
        return false;
    }
    public boolean autonShoot(Follower follower, long timeoutMs) {
        return autonShootByDistance(follower, timeoutMs, getDistanceFromGoal(follower));
    }
}
