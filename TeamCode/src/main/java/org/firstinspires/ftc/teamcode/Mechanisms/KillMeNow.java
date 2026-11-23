package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain2;


@TeleOp
public class KillMeNow extends OpMode {

    // Hardware declarations
    private Limelight3A limelight;
    private final DriveTrain2 drive = new DriveTrain2();

    // PD controller variables
    double kP = 0.002;          // Proportional gain: controls how aggressively the robot turns based on how far off-center it is
    double error = 0;           // Current error: the difference between our goal and the actual target position
    double lastError = 0;       // Previous error: the error from the last loop iteration, used to calculate the derivative term
    double goalX = 0;           // Target horizontal offset (0 = dead center)
    double angleTolerance = 0.4;// Acceptable error margin: how close to center we need to be before stopping the auto-turn
    double kD = 0.0001;         // Derivative gain: damps the turning speed to prevent the robot from overshooting or oscillating back and forth
    double curTime = 0;         // Current time: tracks the timestamp for the current loop cycle
    double prevTime = 0;        // Previous time: timestamp from the last loop cycle, used to find the time change (dt)
    //this is lowk stolen so this is for reference
double forward,strafe,rotate;
    ElapsedTime timer = new ElapsedTime();
double[] stepSizes = {1.0,0.1,0.001,0.0001};
int stepIndex = 2;
    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Fixed: Call initDrivetrain instead of init with a boolean
        drive.initDrivetrain(hardwareMap);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        // Default driving controls (adjust to match your MecanumDrive methods)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();

        // Hold a button (e.g., A) to auto-rotate to the AprilTag
        if (gamepad1.a && result != null && result.isValid()) {
            double tx = result.getTx(); // Horizontal offset in degrees
            curTime = timer.seconds();
            double dt = curTime - prevTime;

            // Calculate PD error
            error = goalX - tx;

            if (Math.abs(error) > angleTolerance) {
                double p = error * kP;
                double d = (dt > 0) ? ((error - lastError) / dt) * kD : 0;

                // Override manual turning with PD output
                turn = -(p + d);
            } else {
                turn = 0;
            }

            lastError = error;
            prevTime = curTime;

            telemetry.addData("Target X", tx);
            telemetry.addData("Error", error);
        } else {
            // Reset error tracking when not actively aligning
            lastError = 0;
            prevTime = timer.seconds();
            telemetry.addData("Limelight", result != null && result.isValid() ? "Tag Detected" : "No Tag");
        }

        // Send movement commands to your drivetrain (check your MecanumDrive method name)
        drive.driveFieldRelative(forward, strafe, turn);
        telemetry.update();
    }
}