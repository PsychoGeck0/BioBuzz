package org.firstinspires.ftc.teamcode.RunPaths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.AutoPaths.Red;
import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain2;

import  org.firstinspires.ftc.teamcode.Auto.AutoPaths.RedV2;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


// =========================
// Motor Power Variables
// =========================

// =========================
// Initialization
// =========================


@Autonomous(name = "RunBlueV2", group = "Autonomous1")

public class RunBlueV2 extends OpMode {
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;
    private Follower follower;
    private final RedV2 paths = new RedV2();
    private final DriveTrain2 driveTrain = new DriveTrain2();

    private DcMotorEx outtake = null;
    private DcMotor intake = null;
    private DcMotor transfer = null;
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
                instant(()-> outtake.setVelocity(1250)),
                Wait(2000),
                instant(()-> transfer.setPower(1)),
                Wait(800),
                instant(()-> outtake.setVelocity(0)),
                instant(()-> transfer.setPower(0)),

                follow(follower, paths.path1()),
                instant(()-> intake.setPower(1)),




                follow(follower, paths.path2()),
                Wait(800),
                instant(()-> intake.setPower(0)),


                follow(follower, paths.path3()),
                instant(()-> outtake.setVelocity(1250)),
                Wait(2000),
                instant(()-> transfer.setPower(1)),
                //add lower flower
                Wait(800),
//add lower hood
                instant(()-> outtake.setVelocity(0)),
                instant(()-> transfer.setPower(0)),

                follow(follower, paths.path4()),
                instant(() -> intake.setPower(1)),
                Wait(800),
                instant(() -> intake.setPower(0)),
                instant(()-> outtake.setVelocity(1250)),
                Wait(2000),

                follow(follower, paths.path5()),
                instant(()-> transfer.setPower(1)),
                Wait(800),


                follow(follower, paths.path6())
                //add transfer when built

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


        // Other motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "launcher");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");




        // Drive directions

        transfer.setDirection(DcMotor.Direction.REVERSE);


        // Brake behavior

        intake.setZeroPowerBehavior(BRAKE);
        transfer.setZeroPowerBehavior(BRAKE);

        // Launcher configuration
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0.05, 0, 0, 12.5)
        );






    }
}

