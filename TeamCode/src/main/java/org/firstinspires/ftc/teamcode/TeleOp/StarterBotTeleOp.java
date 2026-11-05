package org.firstinspires.ftc.teamcode.TeleOp;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "StarterBotTeleOp")

public class StarterBotTeleOp extends OpMode
{

    private DcMotor FrontLeftMotor = null;
    private DcMotor BackLeftMotor = null;
    private DcMotor FrontRightMotor = null;
    private DcMotor BackRightMotor = null;

    private DcMotorEx outtake = null;
    private DcMotor intake = null;

    private DcMotor transfer = null;
    private CRServo angler = null;



    // =========================
    // Launcher Settings
    // =========================

    public final int LAUNCHER_POLLEN_VELOCITY = 1250;
    public final int LAUNCHER_NECTAR_VELOCITY = 1900;

    // =========================
    // Motor Power Variables
    // =========================

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;

    private double transferPower;
    double currentAngle = 0;
    double step = 10;
    double degreesPerPower = 360.0;
    double descent = 10;



    // =========================
    // Initialization
    // =========================

    @Override
    public void init() {

        // Drive motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        angler = hardwareMap.get(CRServo.class, "Angle");


        // Other motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");

        // Servos

        // Drive directions
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        // Brake behavior

        FrontLeftMotor.setZeroPowerBehavior(BRAKE);
        FrontRightMotor.setZeroPowerBehavior(BRAKE);
        BackLeftMotor.setZeroPowerBehavior(BRAKE);
        BackRightMotor.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        transfer.setZeroPowerBehavior(BRAKE);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Launcher configuration
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );
angler.setPower(0);
        // Initialize servos


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Nothing needed here
    }

    @Override
    public void start() {
        // Nothing needed here
    }

    // =========================
    // Main Driver-Controlled Loop
    // =========================

    @Override
    public void loop() {

        // Mecanum drive
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_stick_x
        );

        // Intake controls
        intakePower = gamepad2.right_trigger - gamepad2.left_trigger;


        if (gamepad2.b) {
            transfer.setPower(-1);
        }

        else
        {
            transfer.setPower(0);
        }

        currentAngle += step;
        double power = currentAngle / degreesPerPower;
        degreesPerPower = 0.36;
        currentAngle -= descent;
        double power2 = currentAngle/degreesPerPower;

if (gamepad2.x){
    angler.setPower(Math.max(-1.0, Math.min(1, power)));

}
if (gamepad2.b){
    angler.setPower(Math.max(1.0, Math.min(1, power2)));
}

        // Launcher and windmill
        launch();

        // Apply intake power
        intake.setPower(intakePower);

        // Telemetry
        telemetry.addData(
                "Motors",
                "left (%.2f), right (%.2f)",
                leftFrontPower,
                rightFrontPower
        );

        telemetry.addData(
                "Triggers",
                "left (%.2f), right (%.2f)",
                gamepad1.left_trigger,
                gamepad1.right_trigger
        );
    }

    @Override
    public void stop() {
        // Nothing needed here
    }

    // =========================
    // Mecanum Drive
    // =========================

    void mecanumDrive(double forward, double strafe, double rotate) {

        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        // Normalize motor powers so none exceed 1.0
        double max = Math.max(
                Math.abs(leftFrontPower),
                Math.abs(rightFrontPower)
        );

        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send power to the drive motors
        FrontLeftMotor.setPower(leftFrontPower);
        FrontRightMotor.setPower(rightFrontPower);
        BackLeftMotor.setPower(leftBackPower);
        BackRightMotor.setPower(rightBackPower);

    }


    void launch() {

        // Spin up launcher while holding right bumper
        if (gamepad2.right_bumper)
        {
            outtake.setVelocity(LAUNCHER_POLLEN_VELOCITY);
        } else {
            outtake.setVelocity(0);
        }

        if (gamepad2.left_bumper)
        {
            outtake.setVelocity(LAUNCHER_NECTAR_VELOCITY);
        } else {
            outtake.setVelocity(0);
        }

        // Feed only when launcher reaches minimum velocity


        telemetry.addData(String.valueOf(leftFrontPower), "FrontLeftMotor");
        telemetry.addData(String.valueOf(rightFrontPower), "FrontRightMotor");
        telemetry.addData(String.valueOf(leftBackPower), "BackLeftMotor");
        telemetry.addData(String.valueOf(rightBackPower), "BackRightMotor");
    }

}

