package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Turret;
import com.qualcomm.robotcore.hardware.CRServo;

public class Robot {

    public static final int BLUE_TARGET_TAG_ID = 20;
    public static final int PIPELINE_ID_BLUE = 8;
    public static final int RED_TARGET_TAG_ID = 24;
    public static final int PIPELINE_ID_RED = 2;
    public static int current_pipeline_id = PIPELINE_ID_RED;
    public static int current_tag_id = RED_TARGET_TAG_ID;

    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Robot (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        CRServo turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turret = new Turret(turretServo);  // ✅ CORRECT - passing the CRServo
        this.telemetry = telemetry;
    }
}
