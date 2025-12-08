package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public DcMotorEx intakeMotor;
    private final double INTAKE_POWER = 1.00;

    private final double REVERSE_INTAKE_POWER = -0.75;

    private final double SHOOTING_POWER = 0.5;

    private final double INTAKE_OFF = 0.0;

    private final double TRANSFER_INTAKE_POWER = -0.4;

    private final double TRANFER_SHOOTING_POWER = -0.75;

    private final double TRANSFER_OFF = 0.0;



    public Intake (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }


    public void startIntakeOnly() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void startReverseIntake() {intakeMotor.setPower(REVERSE_INTAKE_POWER);}


    public void intakeStop(){
        intakeMotor.setPower(0);
    }
    public void stopIntake() {
        intakeMotor.setPower(INTAKE_OFF);
    }



}
