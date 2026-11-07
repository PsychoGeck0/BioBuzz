package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FieldCentricDrive extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BackLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BackRightMotor");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
      DcMotorEx outtake = (DcMotorEx) hardwareMap.dcMotor.get( "launcher");
      DcMotorEx transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CRServo angler = hardwareMap.crservo.get("Angle");
        CRServoImplEx flower = (CRServoImplEx) hardwareMap.crservo.get("flower");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);


        transfer.setZeroPowerBehavior(BRAKE);
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean Pressed = gamepad2.a;
            boolean bPressed = gamepad2.b;
            double currentAngle = 0;
            double currentAngle2 = currentAngle;
            double step = 10;
            double degreesPerPower = 360.0;
            double descent = -10;
            double currentFlowerAngle = 0;
            double currentFlowerAngle2 = currentAngle;
            double flowerStep = 30;
            double flowerDegreesPerPower = 360.0;
            double flowerDescent = -30;




            if (gamepad2.left_bumper) {
                intake.setPower(1.0); // Full intake

            } else {
                intake.setPower(0.0);
            }
            if (gamepad2.right_bumper) {
                outtake.setVelocity(4000);
            }
            else if (gamepad2.b){
                outtake.setVelocity(1800);
            }
            else {
                outtake.setVelocity(0);
            }

            if (gamepad2.left_bumper) {
                transfer.setVelocity(-400);
            }

              else if (gamepad2.y){


                  transfer.setVelocity(-1000);
              }
              else {
                  transfer.setVelocity(0);
              }



            currentAngle += step;
            double power = currentAngle / degreesPerPower;
            degreesPerPower = 0.36;
            currentAngle2 -= descent;
            double power2 = currentAngle2/degreesPerPower;

            currentFlowerAngle += flowerStep;
            double powerF = currentFlowerAngle/flowerDegreesPerPower;
            flowerDegreesPerPower = 0.36;
            currentFlowerAngle2 = flowerDescent;
            double power2F = currentFlowerAngle2/flowerDegreesPerPower;


            if (gamepad2.dpad_up){
                angler.setPower(power2);

            }
            if (gamepad2.dpad_down){
                angler.setPower(power);
            }
if (gamepad1.dpad_down){
    flower.setPower(powerF);
}
if (gamepad1.dpad_up){
    flower.setPower(power2F);
}


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }



            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}