package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "AutoPath1", group = "Autonomous")
public class Blue extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(72.7758, 132.9674, 90);
    private final Pose path1 = poseFactory.of(82.5009, 130.5765, 90);
    private final Pose point2 = poseFactory.of(82.3213, 117.7225, 89.1997);
    private final Pose point3Start = poseFactory.of(82.3213, 117.7225, 90);
    private final Pose point3 = poseFactory.of(133.2337, 131.2765, 90);
    private final Pose point4 = poseFactory.of(83.262, 105.0653, 90);
    private final Pose point4Control1 = poseFactory.of(82.945, 126.0971, 0);
    private final Pose point5 = poseFactory.of(95.3273, 21.1503, 180);
    private final Pose point5Control1 = poseFactory.of(140.2216, 94.7887, 0);
    private final Pose point6 = poseFactory.of(83.3, 21.5395, -90);
    private final Pose point7 = poseFactory.of(95.0335, 33.0301, -90);
    private final Pose point8 = poseFactory.of(93.8711, 12.0344, -90);
    private final Pose point9 = poseFactory.of(83.4038, 22.6916, -90);
    private final Pose point10 = poseFactory.of(133.7225, 36.3428, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9()),
                follow(follower, path10())
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
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).linear(point5, point6);
    }

    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }

    public Path path8() {
        return line(point7, point8).linear(point7, point8);
    }

    public Path path9() {
        return line(point8, point9).linear(point8, point9);
    }

    public Path path10() {
        return line(point9, point10).linear(point9, point10);
    }
}
