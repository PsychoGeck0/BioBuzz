package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class CrustySleepTimerCheeksBumBum extends LinearOpMode {
    private DcMotor FrontRightMotor = null;
    private DcMotor FrontLeftMotor = null;
    private DcMotor BackRightMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftMotor.setZeroPowerBehavior(BRAKE);
        FrontRightMotor.setZeroPowerBehavior(BRAKE);
        BackLeftMotor.setZeroPowerBehavior(BRAKE);
        BackRightMotor.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            driveTicks(400) ;


        }


    }


    public void driveTicks(int leftTicks) {
        int rightTicks = leftTicks;
        FrontLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        BackLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        FrontRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);
        BackRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRightMotor.setPower(1);
        FrontLeftMotor.setPower(1);
        BackRightMotor.setPower(1);
        BackLeftMotor.setPower(1);
        while (opModeIsActive() && FrontLeftMotor.isBusy() && FrontRightMotor.isBusy() && BackLeftMotor.isBusy() && BackRightMotor.isBusy()) {
            telemetry.addData("LeftFront", FrontLeftMotor.getCurrentPosition());
            telemetry.addData("BackLeft", BackLeftMotor.getCurrentPosition());
            telemetry.addData("FrontRight", FrontRightMotor.getCurrentPosition());
            telemetry.addData("BackRight", BackRightMotor.getCurrentPosition());
            telemetry.update();

        }


    }

    public void turnTicks(int leftTicks, int rightTicks) {

        FrontLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        BackLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        FrontRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);
        BackRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeftMotor.setPower(1);
        FrontRightMotor.setPower(1);
        BackLeftMotor.setPower(1);
        BackRightMotor.setPower(1);

        while (opModeIsActive() && FrontLeftMotor.isBusy() && FrontRightMotor.isBusy() && BackLeftMotor.isBusy() && BackRightMotor.isBusy()) {
            telemetry.addData("Front Left turn ", FrontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right turn ", FrontRightMotor.getCurrentPosition());
            telemetry.addData("Back Right turn ", BackRightMotor.getCurrentPosition());
            telemetry.addData("Back Left turn ", BackLeftMotor.getCurrentPosition());

            telemetry.update();
        }
    }
    public void StrafeTicks(int leftTicks, int rightTicks){
        FrontLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        BackLeftMotor.setTargetPosition(FrontLeftMotor.getCurrentPosition() + leftTicks);
        FrontRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);
        BackRightMotor.setTargetPosition(FrontRightMotor.getCurrentPosition() + rightTicks);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeftMotor.setPower(1);
        FrontRightMotor.setPower(1);
        BackLeftMotor.setPower(1);
        BackRightMotor.setPower(1);


    }
}

