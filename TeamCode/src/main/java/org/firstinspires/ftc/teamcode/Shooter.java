package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.TreeMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public final int shooterCloseRPM = 1000; //950
    public final int shooterFarRPM = 1515;
    public final int shooterClosePassiveRPM = 1225;
    public final int shooterFarPassiveRPM = 1400;
    public final int autonShooterFarRPM = 1440;
    public final int autonMidRPM = 1250;
    public final int autonMidRedRPM = 1250;
    public final int autonMidBlueRPM = 1400;
    public final int shooterMidRedRPM = 1215;
    public final int shooterMidBlueRPM = 1400;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoCloseRed = 1000;
    public final int autoCloseBlue = 1200;

    public int currentRPM = 0;

    public DcMotorEx shooterMotorFront;
    public DcMotorEx shooterMotorBack;
    public Servo shooterLight;
    private Telemetry telemetry;
    private double lastTargetDistance = 0;
    private final double DISTANCE_UPDATE_THRESHOLD = 2.0; // Inches to move before updating RPM
    private final double RPM_TOLERANCE_LOCK = 50; // More lenient window once already "Ready"
    private boolean wasAtSpeed = false; // Add this variable to the top of your class
    private double speedThreshold = 20; // Max variance allowed for initial shoot
    private double closeShotVariance = 20; // Additional variance allowed for the second and third shoot for close shot.
    private double longShotVariance = 0; // Additional variance allowed for the second and third shoot for long shot.
    private static final TreeMap<Double, Double> SHOOTER_LUT = new TreeMap<>();

    static {
        // FORMAT: SHOOTER_LUT.put(DistanceInInches, TargetRPM);
        //close data points
        SHOOTER_LUT.put(53.0, 1100.0);//
        SHOOTER_LUT.put(63.0, 1100.0);//
        SHOOTER_LUT.put(82.0, 1125.0);//
        SHOOTER_LUT.put(100.0, 1215.0);//
        SHOOTER_LUT.put(115.0, 1200.0);
//        ------------------------------
       //far data points
        SHOOTER_LUT.put(145.0, 1400.0);
        SHOOTER_LUT.put(152.0, 1425.0);
        SHOOTER_LUT.put(167.0, 1515.0);
    }

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
        {200, 0, 0, 20.3},
        // index 1: > 1325
        {200, 0, 0, 19}
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

    public void startShooterOff() {
        setRPM(shooterOffRPM);
    }

    public void startClosePassiveShoot() {
        setRPM(shooterClosePassiveRPM);
    }
    public void startFarPassiveShoot() {
        setRPM(shooterFarPassiveRPM);
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

//    public int getRpmbyDistance(double distance) {
//        // Horner's method: Reduces 10 multiplications & 4 pow calls to just 4 multiplications
//        // y = 4470.38504 - 134.7279x + 1.87252x^2 - 0.0107465x^3 + 0.0000223365x^4
//        double rpm = 4470.38504 + distance * (-134.7279 + distance * (1.87252 + distance * (-0.0107465 + distance * 0.0000223365)));
//        return (int) rpm;
//    }

    public int getRpmbyDistance(double distance) {
        // Handle empty table edge case
        if (SHOOTER_LUT.isEmpty()) return 0;

        // 1. Check if the distance is exactly in our table
        if (SHOOTER_LUT.containsKey(distance)) {
            return SHOOTER_LUT.get(distance).intValue();
        }

        // 2. Find the points immediately below and above the current distance
        Map.Entry<Double, Double> lowEntry = SHOOTER_LUT.floorEntry(distance);
        Map.Entry<Double, Double> highEntry = SHOOTER_LUT.ceilingEntry(distance);

        // 3. Handle out-of-bounds (Clamping)
        if (lowEntry == null) return highEntry.getValue().intValue(); // Too close
        if (highEntry == null) return lowEntry.getValue().intValue();  // Too far

        // 4. Perform Linear Interpolation
        double x1 = lowEntry.getKey();
        double y1 = lowEntry.getValue();
        double x2 = highEntry.getKey();
        double y2 = highEntry.getValue();

        // Formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        double interpolatedRPM = y1 + (distance - x1) * ((y2 - y1) / (x2 - x1));

        return (int) interpolatedRPM;
    }
    //CODE FOR TELEOP, IF WE WANT TO ONLY UPDATE THE SHOOTER RPM WHEN WE MOVE A CERTAIN DISTANCE
    // OR IF THE SHOOTER WAS OFF, UNCOMMENT THIS AND COMMENT OUT THE OTHER startShooterbyDistance() method
//    public void startShooterbyDistance(double distance) {
//        // Only update the target if we moved 2+ inches OR if the shooter was off
//        if (Math.abs(distance - lastTargetDistance) > DISTANCE_UPDATE_THRESHOLD || currentRPM == 0) {
//            int targetRPM = getRpmbyDistance(distance);
//            setRPM(targetRPM);
//            lastTargetDistance = distance;
//        }
//    }

    public void startShooterbyDistance(double distance) {
        // No if-statement, no threshold. Just pure, constant updates.
        int targetRPM = getRpmbyDistance(distance);
        setRPM(targetRPM);

        // We keep this just for telemetry or if you use it elsewhere
        lastTargetDistance = distance;
    }
    public void startRapidShooterByDistance(double distance){setRPM(getRpmbyDistance(distance));}

    public void stopShoot() {
        setRPM(shooterOffRPM);
        shooterLight.setPosition(0);
    }
    public boolean reachedSpeed() {
        double error = Math.abs(getCurrentRPM() - currentRPM);

        // If we were already at speed, use the looser lock.
        // If we weren't, we must get within the tight threshold first.
        double activeThreshold = wasAtSpeed ? RPM_TOLERANCE_LOCK : speedThreshold;

        wasAtSpeed = (error <= activeThreshold);
        return wasAtSpeed;
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