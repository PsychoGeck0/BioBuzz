package org.firstinspires.ftc.teamcode.pedro.examples;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
@Disabled
/** Small demonstration route, not a BIOBUZZ competition autonomous. */
public final class DemoPaths {
    public final Pose start;
    public final Path outbound;
    public final Path home;

    public DemoPaths(boolean mirror) {
        PoseFactory poses = PoseFactory.degrees();
        if (mirror) poses = poses.mirrorX(72);
        start = poses.of(36, 36, 0);
        Pose corner = poses.of(60, 36, 0);
        Pose control = poses.of(72, 60, 90);
        Pose end = poses.of(48, 60, 180);

        // A composite path retains each segment's heading interpolation.
        outbound = path(
                line(start, corner).constant(start),
                curve(corner, control, end).linear(corner, end)
        ).with(Constants.foresightConfig.maxVelocityConstraint.at(24.0));
        home = line(end, start).linear(end, start)
                .with(Constants.foresightConfig.maxVelocityConstraint.at(24.0));
    }
}
