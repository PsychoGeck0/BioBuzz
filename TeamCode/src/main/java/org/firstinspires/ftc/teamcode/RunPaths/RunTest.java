package org.firstinspires.ftc.teamcode.RunPaths;

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

import org.firstinspires.ftc.teamcode.Auto.AutoPaths.Blue;
import  org.firstinspires.ftc.teamcode.Auto.AutoPaths.Test;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous(name = "Test", group = "Autonomous2")

public class RunTest extends OpMode {
    private Follower follower;
    private final Test paths = new Test();

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
    // Same wait pattern as nothing. Add, remove or change Wait(...) lines as needed.
    private Command autoRoutine() {
        return sequential(
                Wait(500),
                follow(follower, paths.path1()),
                follow(follower, paths.path2()),
                follow(follower, paths.path3())


        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.start);
        follower.update();
        telemetry.addLine("Test ready");
        telemetry.update();
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
}


