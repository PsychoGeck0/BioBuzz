package org.firstinspires.ftc.teamcode.RunPaths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain2;

import  org.firstinspires.ftc.teamcode.Auto.AutoPaths.Red;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


// =========================
// Motor Power Variables
// =========================

// =========================
// Initialization
// =========================


@Autonomous(name = "Red", group = "Autonomous2")

public class RunRed extends OpMode {
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;
    private Follower follower;
    private final Red paths = new Red();
    private final DriveTrain2 driveTrain = new DriveTrain2();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotorEx outtake = null;
    private DcMotor intake = null;
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;


    // ---- Actions ----
    // For now this just waits. Put real mechanism code inside the instant(...) lines later.
    private Command Wait(int x) {
        return sequential(
                // instant(() -> shooter.spinUp()),
                // instant(() -> shooter.fire()),
                waitMs(x)
        );
    }


    // ---- Routine ----
    // Same wait pattern as Redbb. Add, remove or change Wait(...) lines as needed.
    private Command autoRoutine() {
        return sequential(
                follow(follower, paths.path1()),
                instant(()-> outtake.setVelocity(1500)),
        Wait(1000),
        // Add transfer

        follow(follower, paths.path1_2()),
                instant(()-> intake.setPower(1)),

                follow(follower, paths.path3()),
               Wait(1000),
                instant(()-> intake.setPower(1)),
                follow(follower, paths.path4()),
                instant(()-> intake.setPower(0)),
                instant(()-> outtake.setVelocity(1500)),
                Wait(1000),
                //add transfer
                follow(follower, paths.path5()),
                instant(()-> intake.setPower(1)),

              follow(follower, paths.path6()),
                Wait(1000),
                instant(()-> intake.setPower(0)),
                follow(follower, paths.path7()),
                instant(()-> outtake.setVelocity(1500)),
                Wait(1000),
                //transfer
                Wait(500),
                instant(()-> outtake.setVelocity(0)),
                follow(follower, paths.path8())


        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.start);
        follower.update();
        telemetry.addLine("Blue ready");
        telemetry.update();
        initDrivetrain(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

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






    }
}

