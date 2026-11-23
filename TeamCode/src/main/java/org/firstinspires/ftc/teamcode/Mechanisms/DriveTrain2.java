package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain2 {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotorEx outtake = null;
    private DcMotor intake = null;

    public Servo anglerServo = null;

    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // =========================
    // Initialization
    // =========================

    public void initDrivetrain(HardwareMap hwMap) {

        // Drive motors (names must match the Driver Station config)
        frontLeftMotor = hwMap.get(DcMotor.class, "FrontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "FrontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "BackLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "BackRightMotor");

        // Angler servo
        anglerServo = hwMap.get(Servo.class, "Angle");

        // Other motors
        intake = hwMap.get(DcMotor.class, "intake");
        outtake = hwMap.get(DcMotorEx.class, "launcher");

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
    }

    // NOTE: despite the name, this is robot-relative (no IMU).
    public void driveFieldRelative(double forward, double strafe, double turn) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) + turn;   // front left
        double v2 = r * Math.sin(robotAngle) - turn;   // front right
        double v3 = r * Math.sin(robotAngle) + turn;   // back left
        double v4 = r * Math.cos(robotAngle) - turn;   // back right

        // Normalize so no wheel exceeds 1.0 (keeps the driving direction correct)
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(v1), Math.abs(v2)),
                        Math.max(Math.abs(v3), Math.abs(v4))));

        frontLeftMotor.setPower(v1 / max);
        frontRightMotor.setPower(v2 / max);
        backLeftMotor.setPower(v3 / max);
        backRightMotor.setPower(v4 / max);
    }
}