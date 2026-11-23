
package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name = "Ts bouta error max")
public class LLDrive extends OpMode {

    private Follower follower;
    private Limelight3A Lemon;

    private DcMotor intake;
    private DcMotorEx outtake;
    private DcMotorEx transfer;

    private CRServo angler;
    private CRServo flower;

    private static final int APRILTAG_PIPELINE = 3;

    private boolean previousOptions = false;

    @Override
    public void init() {

        // Initialize PedroPathing
        follower = Constants.create(hardwareMap);

        // Initialize mechanisms
        intake = hardwareMap.dcMotor.get("intake");

        outtake = hardwareMap.get(
                DcMotorEx.class, "launcher"
        );

        transfer = hardwareMap.get(
                DcMotorEx.class, "transfer"
        );

        angler = hardwareMap.get(
                CRServo.class, "Angle"
        );

        flower = hardwareMap.get(
                CRServo.class, "flower"
        );

        // Motor directions
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor modes
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Existing outtake PIDF
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(12.5, 0, 0, 5)
        );

        // Initialize Limelight
        Lemon = hardwareMap.get(
                Limelight3A.class, "Limelight"
        );

        Lemon.pipelineSwitch(APRILTAG_PIPELINE);

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("PedroPathing initialized");
        telemetry.addLine("Limelight initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        Lemon.start();
    }

    @Override
    public void loop() {

        // FIELD-CENTRIC PEDROPATHING DRIVE
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        // Reset Pedro heading
        if (gamepad1.options && !previousOptions) {
            follower.setHeading(0);
        }

        previousOptions = gamepad1.options;

        follower.manual(powers);
        follower.update();

        // MANUAL MECHANISM CONTROLS

        // Intake
        if (gamepad2.left_bumper) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }

        // Existing outtake presets
        if (gamepad2.right_bumper) {
            outtake.setVelocity(1660);
        } else if (gamepad2.b) {
            outtake.setVelocity(764);
        } else {
            outtake.setVelocity(0);
        }

        // Transfer
        if (gamepad2.left_bumper) {
            transfer.setVelocity(-400);
        } else if (gamepad2.y) {
            transfer.setVelocity(-1000);
        } else {
            transfer.setVelocity(0);
        }

        // Angler
        if (gamepad2.dpad_up) {
            angler.setPower(1);
        } else if (gamepad2.dpad_down) {
            angler.setPower(-1);
        } else {
            angler.setPower(0);
        }

        // Flower
        if (gamepad1.dpad_down) {
            flower.setPower(1);
        } else if (gamepad1.dpad_up) {
            flower.setPower(-1);
        } else {
            flower.setPower(0);
        }

        // LIMELIGHT APRILTAG READINGS

        boolean outtakeRequested =
                gamepad2.right_bumper || gamepad2.b;

        if (outtakeRequested) {

            LLResult result = Lemon.getLatestResult();

            if (result == null || !result.isValid()) {

                telemetry.addLine("No valid Limelight result");

            } else if (result.getPipelineIndex()
                    != APRILTAG_PIPELINE) {

                telemetry.addData(
                        "Limelight",
                        "Waiting for AprilTag pipeline"
                );

            } else {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                telemetry.addData("Tags Found", tags.size());

                if (tags.isEmpty()) {
                    telemetry.addLine("No AprilTags detected");
                }

                for (LLResultTypes.FiducialResult tag : tags) {

                    telemetry.addData(
                            "AprilTag ID",
                            tag.getFiducialId()
                    );

                    telemetry.addData(
                            "TX",
                            tag.getTargetXDegrees()
                    );

                    telemetry.addData(
                            "TY",
                            tag.getTargetYDegrees()
                    );

                    telemetry.addData(
                            "Tag Area",
                            tag.getTargetArea()
                    );

                    telemetry.addData(
                            "Robot Relative Pose",
                            tag.getTargetPoseRobotSpace()
                    );
                }
            }

        } else {

            telemetry.addLine("Vision: Standby");
        }

        // PEDRO LOCALIZATION TELEMETRY

        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());

        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        follower.stop();

        intake.setPower(0);
        outtake.setVelocity(0);
        transfer.setVelocity(0);

        angler.setPower(0);
        flower.setPower(0);

        Lemon.stop();
    }
}
