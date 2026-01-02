package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public final int shooterCloseRPM = 1060; //950
    public final int shooterFarRPM = 1435;
    public final int autonShooterFarRPM = 1370;
    public final int autonMidRPM = 1050;
    public final int shooterMidRPM = 1215;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoClose = 800;

    private int currentRPM = 0;

    public DcMotorEx shooterMotor;
    public Servo shooterLight;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLight = hardwareMap.get(Servo.class, "shooterLight");
        this.telemetry = telemetry;

    }

    public void startCloseShoot() {
        currentRPM = shooterCloseRPM;
        shooterMotor.setVelocity(shooterCloseRPM); // converting RPM to ticks per second
    }

    public void startAutoCloseShoot() {
        currentRPM = autoClose;
        shooterMotor.setVelocity(autoClose); // converting RPM to ticks per second
    }

    public void startAutoMidShoot() {
        currentRPM = autonMidRPM;
        shooterMotor.setVelocity(autonMidRPM); // converting RPM to ticks per second
    }

    public void startAutonFarShoot() {
        currentRPM = autonShooterFarRPM;
        shooterMotor.setVelocity(autonShooterFarRPM);
    }

    public void startFarShoot() {
        currentRPM = shooterFarRPM;
        shooterMotor.setVelocity(shooterFarRPM); // converting RPM to ticks per second
    }

    public void startMidShoot() {
        currentRPM = shooterMidRPM;
        shooterMotor.setVelocity(shooterMidRPM); // converting RPM to ticks per second
    }

    public void startHumanIntake() {
        currentRPM = shooterHumanRPM;
        shooterMotor.setVelocity(shooterHumanRPM); // converting RPM to ticks per second
    }

    public double getCurrentRPM() {
       return shooterMotor.getVelocity();
    }

    public void startTargetShooterSpeed(int newRPM) {
        telemetry.addData("Start Shoot Called", "Yes");
        if (currentRPM != newRPM) {
            if (newRPM >= 1325)  {
                shooterMotor.setVelocityPIDFCoefficients(
                        350.0 ,   // P
                        0,       // I
                        0,       // D
                        20.0   // F
                );
            } else {
                shooterMotor.setVelocityPIDFCoefficients(
                        150 ,   // P
                        0,       // I
                        0,       // D
                        21.3   // F
                );
            }

            shooterMotor.setVelocity(newRPM);
            currentRPM = newRPM;
        }
        telemetry.addData("New Velocity: ", newRPM);
        telemetry.addData("Current Velocity: ", currentRPM);
    }

    public void stopShoot() {
//        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // converting RPM to ticks per second
        currentRPM = shooterOffRPM;
        shooterMotor.setVelocity(shooterOffRPM); // converting RPM to ticks per second
        shooterLight.setPosition(0);
    }

    public boolean reachedSpeed() {
        return Math.abs(shooterMotor.getVelocity() - currentRPM) <= 40;
    }
    public boolean reachMidSpeed () {
        if (shooterMotor.getVelocity() >= shooterMidRPM) {
            return true;
        } else {
            return false;
        }
    }
    public boolean reachAutoMidSpeed () {
        if (shooterMotor.getVelocity() >= autonMidRPM) {
            return true;
        } else {
            return false;
        }
    }
    public boolean reachCloseSpeed () {
        if (shooterMotor.getVelocity() >= shooterCloseRPM) {
            return true;
        } else {
            return false;
        }
    }

    public void shooterLightUpdate() {
        if(currentRPM == 0){
            return;
        }
        if (reachedSpeed()) {
            shooterLight.setPosition(0.5); //sets color to green
        } else {
            shooterLight.setPosition(0.3); //sets color to red
        }
    }



    public void startReverseShoot() {
        shooterMotor.setVelocity(-shooterCloseRPM);
    }

    public void stopFlyWheel() {
        shooterMotor.setVelocity(0);
    }
}