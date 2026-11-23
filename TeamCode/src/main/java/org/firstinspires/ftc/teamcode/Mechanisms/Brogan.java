package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Brogan extends OpMode {

    // Hardware
    private Limelight3A limelight;
    private final DriveTrain2 drive = new DriveTrain2();

    // ---------------------------------------------------------------
    // Auto-align (PD) gains. ALL START AT ZERO: tune them live.
    // ---------------------------------------------------------------
    // kP: power per degree of error.
    //   HOW TO FIND IT: Put the robot ~5 ft from the target, offset so tx is about 10-15 deg.
    //   Hold left trigger and raise kP with D-Pad Right (use the big step first, then the
    //   smaller ones). Stop when the robot turns toward the target briskly but starts to
    //   wobble/oscillate around center, then back off ~20-30%. If it turns AWAY from the
    //   target, the sign is wrong: change error = tx - goalX to goalX - tx.
    double kP = 0;
    // kD: power per (degree/second) of error change. Damps overshoot.
    //   HOW TO FIND IT: Tune kP first. If the robot still overshoots or oscillates, raise kD
    //   with D-Pad Up a little at a time until it settles smoothly without twitching.
    //   If it gets jittery or noisy, kD is too high. Leave it at 0 if kP alone works.
    double kD = 0;
    // kS: minimum power needed to make the robot move at all (static friction).
    //   HOW TO FIND IT: With kP set, notice if the robot stalls and hums just short of center.
    //   Raise kS with Y until it creeps the last degree or two into tolerance, but not so
    //   high that it jitters back and forth across center. Typical result is small.
    double kS = 0;
    double goalX = 0;            // target horizontal offset (0 = centered)
    double angleTolerance = 0.4; // degrees; stop turning inside this
    double maxRotate = 0.4;      // rotation power cap

    double error = 0;
    double lastError = 0;
    double prevTime = 0;

    // ---------------------------------------------------------------
    // Auto-angler. Servo pos = anglerOffset + anglerGain * ty.
    // Gain of 0 means the servo is NOT commanded until you calibrate.
    // ---------------------------------------------------------------
    // HOW TO FIND BOTH (needs 2+ calibration shots):
    //   1. Park at a close shot distance. Use the bumpers to set the servo to the position
    //      that scores reliably. Hold left trigger (aimed at the target) and read ty from
    //      telemetry. Write down: ty1, servoPos1.
    //   2. Move to a far shot distance and repeat. Write down: ty2, servoPos2.
    //   3. anglerGain   = (servoPos2 - servoPos1) / (ty2 - ty1)
    //      anglerOffset = servoPos1 - anglerGain * ty1
    //   Check a third distance in between; if it misses, the relationship isn't linear and
    //   you should use a lookup table that interpolates between measured points instead.
    // anglerOffset: servo position when ty = 0.
    double anglerOffset = 0;
    // anglerGain: change in servo position per degree of ty (can be negative).
    double anglerGain = 0;

    // -1 = use whatever Limelight picks as the primary target.
    // Set to a specific AprilTag ID to ignore all other tags.
    int TARGET_TAG_ID = -1;

    // Max age (ms) of a Limelight result before we ignore it
    static final long MAX_STALENESS_MS = 100;

    double forward, strafe, rotate;
    ElapsedTime timer = new ElapsedTime();

    // Live tuning: B cycles step size, D-pad changes kP/kD, X/Y change kS
    double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 1;

    boolean lastBState = false;
    boolean lastXState = false;
    boolean lastYState = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);   // make sure pipeline 0 is set up in the Limelight web UI
        limelight.start();

        drive.initDrivetrain(hardwareMap);
    }

    @Override
    public void start() {
        timer.reset();
        prevTime = timer.seconds();
    }

    @Override
    public void loop() {
        double now = timer.seconds();
        double dT = now - prevTime;

        // Manual driving inputs
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // 1. Auto-align + auto-angle while holding left trigger
        if (gamepad1.left_trigger > 0.3) {
            double[] target = getTarget();   // {tx, ty} or null

            if (target != null) {
                double tx = target[0];
                double ty = target[1];

                // --- TURNING ---
                // tx > 0 means target is to the right; positive rotate = clockwise.
                // If the robot turns AWAY from the target, change this to goalX - tx.
                error = tx - goalX;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double derivative = (dT > 0 && dT < 0.1) ? (error - lastError) / dT : 0;
                    double out = kP * error + kD * derivative;
                    out += Math.signum(out) * kS;
                    rotate = Range.clip(out, -maxRotate, maxRotate);
                }

                lastError = error;
                prevTime = now;

                // --- ANGLING (only once calibrated) ---
                if (anglerGain != 0) {
                    double pos = Range.clip(anglerOffset + anglerGain * ty, 0.0, 1.0);
                    drive.anglerServo.setPosition(pos);
                    telemetry.addData("Angler Servo Pos", pos);
                } else {
                    telemetry.addLine("Angler not calibrated (anglerGain = 0)");
                }

                telemetry.addLine("AUTO ALIGN ACTIVE");
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Rotate cmd", rotate);
            } else {
                // Lost target: reset derivative state so it doesn't spike on re-acquire
                lastError = 0;
                prevTime = now;
                telemetry.addLine("AUTO ALIGN: No valid target");
            }
        } else {
            telemetry.addLine("MANUAL MODE");
            lastError = 0;
            prevTime = now;

            // Manual angler with bumpers
            if (gamepad1.right_bumper) {
                double newPos = Range.clip(drive.anglerServo.getPosition() + 0.01, 0.0, 1.0);
                drive.anglerServo.setPosition(newPos);
            } else if (gamepad1.left_bumper) {
                double newPos = Range.clip(drive.anglerServo.getPosition() - 0.01, 0.0, 1.0);
                drive.anglerServo.setPosition(newPos);
            }
        }

        // 2. Live tuning
        boolean currentB = gamepad1.b;
        if (currentB && !lastBState) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastBState = currentB;

        if (gamepad1.dpad_left && !lastDpadLeft)   { kP = Math.max(0, kP - stepSizes[stepIndex]); }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) { kP += stepSizes[stepIndex]; }
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad1.dpad_up && !lastDpadUp)       { kD += stepSizes[stepIndex]; }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown)   { kD = Math.max(0, kD - stepSizes[stepIndex]); }
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.y && !lastYState)             { kS += stepSizes[stepIndex]; }
        lastYState = gamepad1.y;

        if (gamepad1.x && !lastXState)             { kS = Math.max(0, kS - stepSizes[stepIndex]); }
        lastXState = gamepad1.x;

        // Drive
        drive.driveFieldRelative(forward, strafe, rotate);

        // 3. Telemetry
        telemetry.addLine("--------------------------------");
        telemetry.addData("kP (D-Pad L/R)", "%.5f", kP);
        telemetry.addData("kD (D-Pad U/D)", "%.5f", kD);
        telemetry.addData("kS (X down / Y up)", "%.5f", kS);
        telemetry.addData("Step size (B)", "%.5f", stepSizes[stepIndex]);
        telemetry.update();
    }

    /**
     * Returns {tx, ty} for the target, or null if there is no fresh, valid one.
     * If TARGET_TAG_ID >= 0, only that AprilTag is used.
     */
    private double[] getTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            return null;
        }

        if (TARGET_TAG_ID < 0) {
            return new double[]{result.getTx(), result.getTy()};
        }

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == TARGET_TAG_ID) {
                return new double[]{tag.getTargetXDegrees(), tag.getTargetYDegrees()};
            }
        }
        return null;
    }
}