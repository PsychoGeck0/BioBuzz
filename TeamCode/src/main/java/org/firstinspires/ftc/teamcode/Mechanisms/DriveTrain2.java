
package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
public class DriveTrain2 {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotorEx outtake = null;
    private DcMotor intake = null;




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

    public void initDrivetrain(HardwareMap hwMap){

        // Drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor =hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Other motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");




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


    }

}