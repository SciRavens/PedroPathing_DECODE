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
    public static int current_pipeline_id = PIPELINE_ID_BLUE;
    public static int current_tag_id = BLUE_TARGET_TAG_ID;
    public static final int RED_GOAL_X = 144;
    public static final int RED_GOAL_Y = 144;
    public static final int BLUE_GOAL_X = 0;
    public static final int BLUE_GOAL_Y = 144;
    public static int current_goal_x = BLUE_GOAL_X;
    public static int current_goal_y = BLUE_GOAL_Y;

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
    }


    public double getDistanceFromGoal(Follower follower) {
        double dx = Robot.current_goal_x - follower.getPose().getX();
        double dy = Robot.current_goal_y - follower.getPose().getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    public boolean autonShootByDistance(Follower follower, long timeoutMs, double distance) {
        gate.gateOpen();
        shooter.startShooterbyDistance(distance);
        if (shooter.reachedSpeed()) {
            if (!is_autonShootTimerOn) {
               shootTimer.resetTimer();
               is_autonShootTimerOn = true;
            }
            intake.startIntake();
        } else {
            intake.stopIntake();
        }
        if (is_autonShootTimerOn && shootTimer.getElapsedTime() > timeoutMs) {
            intake.stopIntake();
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
