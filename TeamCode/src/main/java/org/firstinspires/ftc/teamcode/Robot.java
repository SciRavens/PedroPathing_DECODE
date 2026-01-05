package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public static final int BLUE_TARGET_TAG_ID = 20;
    public static final int PIPELINE_ID_BLUE = 8;
    public static final int RED_TARGET_TAG_ID = 24;
    public static final int PIPELINE_ID_RED = 2;
    public static int current_pipeline_id = PIPELINE_ID_RED;
    public static int current_tag_id = RED_TARGET_TAG_ID;
    public static final int RED_GOAL_X = 144;
    public static final int RED_GOAL_Y = 144;
    public static final int BLUE_GOAL_X = 0;
    public static final int BLUE_GOAL_Y = -144;
    public static int current_goal_x = RED_GOAL_X;
    public static int current_goal_y = RED_GOAL_Y;




    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Gate gate;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Robot (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }
}
