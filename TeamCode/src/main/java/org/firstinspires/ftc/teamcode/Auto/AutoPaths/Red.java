package org.firstinspires.ftc.teamcode.Auto.AutoPaths;

import static com.pedropathing.api.Paths.*;

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

@Autonomous(name = "AutoPath2", group = "Autonomous")
public class Red extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81.5284, 8.7294, 90);
    private final Pose point1Start = poseFactory.of(81.5284, 8.7294, -90);
    private final Pose point1 = poseFactory.of(59.4038, 24.3196, -90);
    private final Pose path1 = poseFactory.of(10.2113, 18.9518, -90);
    private final Pose point3 = poseFactory.of(8.2311, 9.4149, -90);
    private final Pose point4 = poseFactory.of(59.5662, 116.932, 90);
    private final Pose point4Control1 = poseFactory.of(31.5816, 139.2002, 0);
    private final Pose point5 = poseFactory.of(47.8904, 117.2045, 90);
    private final Pose point6 = poseFactory.of(47.6529, 129.5746, 91.0997);
    private final Pose point7Start = poseFactory.of(47.6529, 129.5746, 90);
    private final Pose point7 = poseFactory.of(59.4948, 117.4141, 90);
    private final Pose point8 = poseFactory.of(8.4613, 106.8136, 0);
    private final Pose point8Control1 = poseFactory.of(37.4948, 101.3909, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path1_2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8())
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
        return line(point1Start, point1).linear(point1Start, point1);
    }

    public Path path1_2() {
        return line(point1, path1).linear(point1, path1);
    }

    public Path path3() {
        return line(path1, point3).constant(point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).tangent();
    }

    public Path path7() {
        return line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).linear(point7, point8);
    }
}
