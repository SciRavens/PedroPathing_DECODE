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

    private double speedThreshold = 20; // Max variance allowed for initial shoot
    private double closeShotVariance = 20; // Additional variance allowed for the second and third shoot for close shot.
    private double longShotVariance = 0; // Additional variance allowed for the second and third shoot for long shot.

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
        {400, 0, 0, 18.5}
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

//    public int getRpmbyDistance(double distance) {
//        //        y=0.0000223365x^{4}-0.0107465x^{3}+1.87252x^{2}-134.7279x+4470.38504
//        double rpm = 0.0000223365 * Math.pow(distance, 4)
//                - 0.0107465 * Math.pow(distance, 3)
//                + 1.87252 * Math.pow(distance, 2)
//                - 134.7279 * distance + 4470.38504;
//        return (int) rpm;
//    }

    public int getRpmbyDistance(double distance) {
        // Horner's method: Reduces 10 multiplications & 4 pow calls to just 4 multiplications
        // y = 4470.38504 - 134.7279x + 1.87252x^2 - 0.0107465x^3 + 0.0000223365x^4
        double rpm = 4470.38504 + distance * (-134.7279 + distance * (1.87252 + distance * (-0.0107465 + distance * 0.0000223365)));
        return (int) rpm;
    }
    public void startShooterbyDistance(double distance) {
        setRPM(getRpmbyDistance(distance));
    }

    public void stopShoot() {
        setRPM(shooterOffRPM);
        shooterLight.setPosition(0);
    }

    public boolean reachedSpeed() {
        return Math.abs(getCurrentRPM() - currentRPM) <= speedThreshold;
    }

    public boolean isSafeToContinueShooting(double distance) {
        double dynThreshold = speedThreshold; // Default
        if (distance < 130) { // 8ft
            dynThreshold = speedThreshold + closeShotVariance; // Add additional variance allowed for close shot
        } else {
            dynThreshold = speedThreshold + longShotVariance; // additional variance allowed for long shot
        }
        return Math.abs(getCurrentRPM() - currentRPM) <= dynThreshold;
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