package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    public DcMotorEx intakeMotor;
    private Telemetry telemetry;
    private final double INTAKE_POWER = 1.0;
    private final double REVERSE_INTAKE_POWER = -0.75;
    private final double SHOOTING_POWER = 0.5;
    private final double INTAKE_OFF = 0.0;
    private final double TRANSFER_INTAKE_POWER = -0.4;
    private final double TRANFER_SHOOTING_POWER = -0.75;
    private final double TRANSFER_OFF = 0.0;

    public Intake (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        //intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    public void startReverseIntake() {intakeMotor.setPower(REVERSE_INTAKE_POWER);}

    public void stopIntake() {
        intakeMotor.setPower(INTAKE_OFF);
    }

    public void feedBalls() { startIntake();}
    public void stopFeeding() { stopIntake();}
}
