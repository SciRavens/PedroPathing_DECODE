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
    public final int autoCloseBlue = 1050;

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

    public void setRPM(int rpm) {
        shooterMotorFront.setVelocity(rpm); // converting RPM to ticks per second
        currentRPM = rpm;
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
        if (currentRPM != newRPM) {
            if (newRPM >= 1325) {
                shooterMotorFront.setVelocityPIDFCoefficients(
                        300,   // P
                        0,       // I
                        0,       // D
                        20.3   // F
                );
            }
            else {
                shooterMotorFront.setVelocityPIDFCoefficients(
                        450 ,   // P
                        0,       // I
                        0,       // D
                        18.7   // F
                );
            }

            setRPM(newRPM);
        }
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