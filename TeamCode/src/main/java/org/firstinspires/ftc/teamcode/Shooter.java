package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public final int shooterCloseRPM = 1000; //950
    public final int shooterFarRPM = 1515;
    public final int shooterPassiveRPM = 500;
    public final int autonShooterFarRPM = 1470;
    public final int autonMidRPM = 1250;
    public final int autonMidRedRPM = 1250;
    public final int autonMidBlueRPM = 1400;
    public final int shooterMidRedRPM = 1215;
    public final int shooterMidBlueRPM = 1400;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoCloseRed = 1000;
    public final int autoCloseBlue = 1150;

    private int currentRPM = 0;

    public DcMotorEx shooterMotorFront;
    public DcMotorEx shooterMotorBack;
    public Servo shooterLight;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotorFront = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        shooterMotorFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLight = hardwareMap.get(Servo.class, "shooterLight");
        this.telemetry = telemetry;

    }

    private int curVelocityCoefficient = 0;

    // PID coefficients: [][P, I, D, F]
    private static final double[][] VELOCITY_PID_COEFFICIENTS = {
        // index 0: <= 1325
        {250, 0, 0, 20.3},
        // index 1: > 1325
        {250, 0, 0, 20.3}
    };

    private int getCoefficientIndex(int rpm) {
        return (rpm > 1325) ? 1 : 0;
    }

    public void setRPM(int newRPM) {
        int index = getCoefficientIndex(newRPM);
        if (index != curVelocityCoefficient) {
            shooterMotorFront.setVelocityPIDFCoefficients(
                VELOCITY_PID_COEFFICIENTS[index][0],
                VELOCITY_PID_COEFFICIENTS[index][1],
                VELOCITY_PID_COEFFICIENTS[index][2],
                VELOCITY_PID_COEFFICIENTS[index][3]
            );
            curVelocityCoefficient = index;
        }
        shooterMotorFront.setVelocity(newRPM); // converting RPM to ticks per second
        currentRPM = newRPM;
    }

    public void startAutoCloseRedShoot() {
        setRPM(autoCloseRed);
    }
    public void startAutoCloseBlueShoot() {
        setRPM(autoCloseBlue);
    }

    public void startAutoMidRedShoot() {
        setRPM(autonMidRedRPM);
    }
    public void startAutoMidBlueShoot() {
        setRPM(autonMidBlueRPM);
    }

    public void startAutonFarShoot() {
        setRPM(autonShooterFarRPM);
    }
    public void startMidRedShoot() {
        setRPM(shooterMidRedRPM);
    }

    public void startHumanIntake() {
        setRPM(shooterHumanRPM);
    }

    public void startPassiveShoot() {
        setRPM(shooterPassiveRPM);
    }

    public double getCurrentRPM() {
       return shooterMotorFront.getVelocity();
    }

    public void startTargetShooterSpeed(int newRPM) {
        setRPM(newRPM);
    }
    private int getRpmbyDistance(double distance) {
        // y=-0.00000689201x^{4}+0.00288448x^{3}-0.422368x^{2}+29.95317x+292.88591
        double rpm = -0.00000689201 * Math.pow(distance, 4)
                + 0.00288448 * Math.pow(distance, 3)
                - 0.422368 * Math.pow(distance, 2)
                + 29.95317 * distance + 292.88591;
        return (int)rpm;
    }
    public void startShooterbyDistance(double distance) {
        setRPM(getRpmbyDistance(distance);
    }

    public void stopShoot() {
        setRPM(shooterOffRPM);
        shooterLight.setPosition(0);
    }

    public boolean reachedSpeed() {
        return Math.abs(getCurrentRPM() - currentRPM) <= 20;
    }

    public void shooterLightUpdate() {
        if(currentRPM > 0) {
            if (reachedSpeed()) {
                shooterLight.setPosition(0.5); //sets color to green
            } else {
                shooterLight.setPosition(0.3); //sets color to red
            }
        }
    }
    public void startReverseShoot() {
        setRPM(-shooterCloseRPM);
    }
}