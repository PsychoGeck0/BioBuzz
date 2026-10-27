package org.firstinspires.ftc.teamcode.TeleOp;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotorEx outtake = null;
    private DcMotor intake = null;


    private CRServo windmillServo = null;

    // =========================
    // Launcher Settings
    // =========================

    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // =========================
    // Motor Power Variables
    // =========================

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;

    // =========================
    // Initialization
    // =========================

    @Override
    public void init() {

        // Drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Other motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");

        // Servos
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");


        // Drive directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake behavior
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Launcher configuration
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

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
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Intake controls
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

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
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }


    void launch() {

        // Spin up launcher while holding right bumper
        if (gamepad2.right_trigger_pressed) {
            outtake.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else
        {
            outtake.setVelocity(0);
        }

        // Feed only when launcher reaches minimum velocity
        if (gamepad2.left_bumper
                && outtake.getVelocity() > LAUNCHER_MIN_VELOCITY) {

            windmillServo.setPower(-1);
            intakePower += 0.5;

        } else {
            windmillServo.setPower(0);
        }
    }
}


