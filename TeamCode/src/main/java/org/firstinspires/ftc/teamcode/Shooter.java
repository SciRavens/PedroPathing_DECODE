package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.TreeMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    // --- TUNED CONSTANTS ---
    public static final double P = 0.01;
    public static final double kS = 0.152;
    public static final double kV = 0.0004025;

    // Set this to the battery voltage you had while tuning (usually ~13.0)
    public static final double NOMINAL_VOLTAGE = 12.58;
    private final double RPM_TOLERANCE_LOCK = 50; // More lenient window once already "Ready"


    private VoltageSensor batteryVoltageSensor;
    public DcMotorEx shooterMotorFront;
    //public DcMotorEx shooterMotorBack;
    private Telemetry telemetry;




    public final int shooterCloseRPM = 1000; //950
    public final int shooterFarRPM = 1325;
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
    public final int autoCloseBlue = 1110;

    public double targetRPM = 0;
    public double currentRPM = 0;

    public Servo shooterLight;
    private final double DISTANCE_UPDATE_THRESHOLD = 2.0; // Inches to move before updating RPM
    private boolean wasAtSpeed = false; // Add this variable to the top of your class
    private final double speedThreshold = 20; // Max variance allowed for initial shoot
    private final double closeShotVariance = 20; // Additional variance allowed for the second and third shoot for close shot.
    private final double longShotVariance = 0; // Additional variance allowed for the second and third shoot for long shot.
    private static final TreeMap<Double, Double> SHOOTER_LUT = new TreeMap<>();

    static {
        // FORMAT: SHOOTER_LUT.put(DistanceInInches, TargetRPM);
        //close data points
        SHOOTER_LUT.put(71.0, 1085.0);
        SHOOTER_LUT.put(101.0, 1150.0);
        SHOOTER_LUT.put(107.0, 1150.0);
        SHOOTER_LUT.put(60.0, 1025.0);
        SHOOTER_LUT.put(95.0, 1150.0);
        SHOOTER_LUT.put(84.0, 1100.0);
        SHOOTER_LUT.put(111.25, 1200.0);
        SHOOTER_LUT.put(121.55, 1275.0);
        SHOOTER_LUT.put(76.43, 1085.0);
        SHOOTER_LUT.put(67.74, 1075.0);
//        ------------------------------
       //far data points
        SHOOTER_LUT.put(145.0, 1375.0);
        SHOOTER_LUT.put(152.0, 1400.0);
        SHOOTER_LUT.put(167.0, 1500.0);
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotorFront = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        shooterMotorFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLight = hardwareMap.get(Servo.class, "shooterLight");
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.telemetry = telemetry;
    }

    public void update(Robot robot) {
        robot.turret.update();
        currentRPM = shooterMotorFront.getVelocity();

        if (targetRPM <= 0) {
            shooterMotorFront.setPower(0);
            return;
        }

        // 1. Calculate Error
        double error = targetRPM - currentRPM;

        // 2. Calculate Power using the formula:
        // Power = (kV * target) + kS + (P * error)
        double feedforward = (targetRPM * kV) + kS;
        double feedback = error * P;
        double totalPower = feedforward + feedback;

        // 3. Voltage Compensation
        // Adjusts power based on battery sag (13V nominal)
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltageSensor.getVoltage();

        // 4. Set final motor power (Clamped between 0 and 1)
        double finalPower = Math.max(0, Math.min(1.0, totalPower * voltageComp));
        shooterMotorFront.setPower(finalPower);
        shooterLightUpdate();
    }



    public void setRPM(int newRPM) {
        this.targetRPM = newRPM;
    }

    public void startAutoCloseRedShoot() {
        setRPM(autoCloseRed);
    }
    public void startAutoCloseBlueShoot() {
        setRPM(autoCloseBlue);
    }

    public void startAutonFarShoot() {
        setRPM(autonShooterFarRPM);
    }

    public void startClosePassiveShoot() {
        setRPM(shooterClosePassiveRPM);
    }
    public void startFarPassiveShoot() {
        setRPM(shooterFarPassiveRPM);
    }

    public double getCurrentRPM() {
       return currentRPM;
    }

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

    public void startShooterbyDistance(double distance) {
        setRPM(getRpmbyDistance(distance));
    }

    public void stopShoot() {
        setRPM(shooterOffRPM);
        shooterLight.setPosition(0);
    }

    public boolean reachedSpeed() {
        double error = Math.abs(currentRPM - targetRPM);
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
        return Math.abs(getCurrentRPM() - targetRPM) <= dynThreshold;
    }

    private void shooterLightUpdate() {
        if (targetRPM > 0) {
            shooterLight.setPosition(reachedSpeed() ? 0.5 : 0.3); // Green if ready, Red if spooling
        } else {
            shooterLight.setPosition(0);
        }
    }
}