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

@Autonomous(name = "AutoPath1", group = "Autonomous")
public class Blue extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(83.3717, 132.4376, -90);
    private final Pose point1 = poseFactory.of(133.7635, 123.1529, 90);
    private final Pose point2Start = poseFactory.of(133.7635, 123.1529, 90);
    private final Pose point2 = poseFactory.of(134.6821, 132.6328, 90);
    private final Pose point3Start = poseFactory.of(134.6821, 132.6328, 90);
    private final Pose point3 = poseFactory.of(83.4952, 22.3865, -90);
    private final Pose point3Control1 = poseFactory.of(100.4869, 36.1578, 0);
    private final Pose point3Control2 = poseFactory.of(118.3593, 4.579, 0);
    private final Pose point4Start = poseFactory.of(83.4952, 22.3865, -90);
    private final Pose point4 = poseFactory.of(94.5775, 12.7408, -90);
    private final Pose point5Start = poseFactory.of(94.5775, 12.7408, -90);
    private final Pose point5 = poseFactory.of(83.4038, 22.6916, 90);
    private final Pose point6Start = poseFactory.of(83.4038, 22.6916, 90);
    private final Pose point6 = poseFactory.of(133.7225, 36.3428, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
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
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }
}
