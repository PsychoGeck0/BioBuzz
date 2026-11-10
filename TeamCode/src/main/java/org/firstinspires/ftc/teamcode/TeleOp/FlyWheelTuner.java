package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlyWheelTuner extends OpMode {

    public DcMotorEx outtake;

    double highVelocity = 720;
    double lowVelocity = 700;

    double targetVelocity = highVelocity;

    double F = 0;
    double P = 0;double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoeffcients = new PIDFCoefficients(P , 0, 0, F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffcients);
        telemetry.addLine("Init Complete");


    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed())
        {
            if(targetVelocity == highVelocity)
            {
                targetVelocity = lowVelocity;
            }
            else {
                targetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed())
        {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed())
        {
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed())
        {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed())
        {
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed())
        {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients.pidfCoefficents

    }
}
