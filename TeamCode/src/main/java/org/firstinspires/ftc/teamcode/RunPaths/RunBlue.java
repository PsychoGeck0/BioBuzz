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

import  org.firstinspires.ftc.teamcode.Auto.AutoPaths.Blue;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous(name = "Blue", group = "Autonomous1")

public class RunBlue extends OpMode {
    private Follower follower;
    private final Blue paths = new Blue();

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
                Wait(500),
                follow(follower, paths.path1()),
                follow(follower, paths.path2()),
                Wait(500),
                follow(follower, paths.path3()),
                Wait(1000),
                follow(follower, paths.path4()),
                follow(follower, paths.path5()),
                Wait(1000),
                follow(follower, paths.path6()),
                Wait(500),
                follow(follower, paths.path7()),
                follow(follower, paths.path8()),
                follow(follower, paths.path9()),
                follow(follower, paths.path10())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.start);
        follower.update();
        telemetry.addLine("BlueBB ready");
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

