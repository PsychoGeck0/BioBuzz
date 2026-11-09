package org.firstinspires.ftc.teamcode.Auto.AutoPaths;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BlueV2", group = "Autonomous")
public class BlueV2 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(29.5101, 11.9825, 90);
    private final Pose point2Start = poseFactory.of(29.5101, 11.9825, 90);
    private final Pose point2 = poseFactory.of(30.1063, 46.113, 90);
    private final Pose point3Start = poseFactory.of(30.1063, 46.113, 180);
    private final Pose point3 = poseFactory.of(9.9731, 46.8389, 180);
    private final Pose point4Start = poseFactory.of(9.9731, 46.8389, 180);
    private final Pose point4 = poseFactory.of(59.1474, 117.7214, 270);
    private final Pose point5Start = poseFactory.of(59.1474, 117.7214, 90);
    private final Pose point5 = poseFactory.of(46.7818, 132.2366, 90);
    private final Pose point6Start = poseFactory.of(46.7818, 132.2366, 90);
    private final Pose point6 = poseFactory.of(59.1154, 117.7695, 270);
    private final Pose point6Control1 = poseFactory.of(58.342, 104.9944, 0);
    private final Pose point7Start = poseFactory.of(59.1154, 117.7695, 0);
    private final Pose point7 = poseFactory.of(7.4657, 107.8966, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }
}
