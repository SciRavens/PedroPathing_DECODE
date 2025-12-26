package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel Tuner", group = "Experimental")
public class FlywheelTuner extends OpMode {
    public DcMotorEx shooterMotor;
    double shooterCloseRPM = 1375.0;
    double shooterFarRPM = 970.0;
    double currTargetVelocity = shooterFarRPM;
    double F = 0.0;
    double P = 0.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotor.setVelocity(currTargetVelocity);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0.0, 0.0, F);
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Flywheel Tuner Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()){
            if (currTargetVelocity == shooterFarRPM) {
                currTargetVelocity = shooterCloseRPM;
            } else {
                currTargetVelocity = shooterFarRPM;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0.0, 0.0, F);
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooterMotor.setVelocity(currTargetVelocity);
        double currentVelocity = shooterMotor.getVelocity();
        double error = currTargetVelocity - currentVelocity;

        telemetry.addData("Target Velocity (RPM): ", currTargetVelocity);
        telemetry.addData("Current Velocity (RPM): ", currentVelocity);
        telemetry.addData("Error (RPM): ", error);
        telemetry.addData("P: ", P);
        telemetry.addData("F: ", F);
        telemetry.addData("Step Size: ", stepSizes[stepIndex]);
        telemetry.update();

    }
    }
