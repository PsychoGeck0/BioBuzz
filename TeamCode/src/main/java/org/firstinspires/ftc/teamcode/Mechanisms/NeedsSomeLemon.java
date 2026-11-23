package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous
public class NeedsSomeLemon extends OpMode {
    private Limelight3A Lemon;
    private IMU IamMoo;
    @Override
    public void init() {
Lemon = hardwareMap.get(Limelight3A.class, "Limelight");
Lemon.pipelineSwitch(0); //I have no flying idea if this is the right Ps
        IamMoo = hardwareMap.get(IMU.class, "IMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
    }

    @Override
    public void start() {
        Lemon.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = IamMoo.getRobotYawPitchRollAngles();
        Lemon.updateRobotOrientation(orientation.getYaw());
        LLResult LLResult = Lemon.getLatestResult();
        if (LLResult != null && LLResult.isValid()){
            Pose3D BotPosition = LLResult.getBotpose_MT2();
            telemetry.addData("TargetX", LLResult.getTx());
            telemetry.addData("TargetY",LLResult.getTy());
            telemetry.addData("TargetA",LLResult.getTa());
            telemetry.addData("Yaw", BotPosition.getOrientation().getYaw());
        }
    }
}
