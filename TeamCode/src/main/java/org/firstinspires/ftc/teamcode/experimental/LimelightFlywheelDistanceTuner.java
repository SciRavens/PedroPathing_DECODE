//package org.firstinspires.ftc.teamcode.experimental;
//import org.firstinspires.ftc.teamcode.Robot;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import java.util.List;
//public class LimelightFlywheelDistanceTuner extends OpMode {
//    public DcMotorEx shooterMotor;
//    private Limelight3A limelight;
//    private GoBildaPinpointDriver pinpoint;
//
//
//    @Override
//    public void init() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(Robot.current_pipeline_id);
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//    }
//
//    public void start() {
//        limelight.start();
//    }
//
//    @Override
//    public void loop() {
//        YawPitchRollAngles orientation = pinpoint.getImuYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llresult = limelight.getLatestResult();
//        if (llresult != null && llresult.isValid()) {
//            Pose3D botPose = llresult.getBotpose_MT2();
//            telemetry.addData("Tx", llresult.getTx());
//            telemetry.addData("Ta", llresult.getTa());
//            telemetry.addData("BotPose", botPose.toString());
//        }
//    }
//}
