package org.firstinspires.ftc.teamcode.TeleOp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.telecom.TelecomManager;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
@Configurable
public class FlyWheelTuner extends OpMode {
    TelemetryManager panelsTelemetry;

    public DcMotorEx outtake;
    public DcMotorEx transfer;
    public CRServo angler;

    public static double F = 0;
    public static double P = 150;

    double highVelocity = 720;
    double lowVelocity = 700;

    double targetVelocity = highVelocity;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        angler = hardwareMap.get(CRServo.class, "Angle");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P , 0, 0, F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
        if(gamepad1.dpadDownWasPressed())
        {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.right_bumper){
            transfer.setVelocity(-1000);
        }
        if (gamepad1.a){
            angler.setPower(1);
        }
        if (gamepad1.x){
            angler.setPower(-1);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P , 0, 0, F);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        outtake.setVelocity(targetVelocity);

        double velocity = outtake.getVelocity();
        double error = targetVelocity - velocity;

        panelsTelemetry.addData("Target Velocity", targetVelocity);
        panelsTelemetry.addData("Current Velocity", velocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        panelsTelemetry.update(telemetry);


    }
}
