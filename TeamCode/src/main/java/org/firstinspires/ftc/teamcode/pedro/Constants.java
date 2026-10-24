package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // Starter names/directions: replace with the Mecanum AutoTune output.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FrontLeftMotor");
        c.frontRightName.set("FrontRightMotor");
        c.backLeftName.set("BackLeftMotor");
        c.backRightName.set("BackRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Replace with Pinpoint AutoTune output, including pod type, offsets and directions.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(6.6081315889133245);
        c.yPodOffset.set(-6.082860991710754);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Library defaults are a starting point; replace with Foresight AutoTune output.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19491130652916844);
                Controller secondaryTranslationalForward = Controller.proportional(0.07201451963649785);
                Controller primaryTranslationalLateral = Controller.proportional(0.25238632888971513);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09325000463783786);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017056393626623985));
                c.brake.set(Controller.proportionalFeedforward(0.014497934582630387));

                c.headingFeedback.set(Controller.proportional(2.8588966993443825));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04833572403226152, 0.00433602080161131));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09939024591195796, 0.038529381959315864));
                c.quadraticBrakeCoefficients.set(Matrix.diag(3.1481542535952465E-4, 0.0018346747190531062));

                c.maxAchievableForwardVelocity.set(57.66022615946626);
                c.maxAchievableStrafeVelocity.set(48.658962895753746);
                c.naturalForwardDeceleration.set(35.921973113989466);
                c.naturalStrafeDeceleration.set(58.97937023840058);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
