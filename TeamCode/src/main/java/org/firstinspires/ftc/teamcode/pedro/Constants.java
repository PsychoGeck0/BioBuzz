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
        c.xPodOffset.set(6.3009721650851995);
        c.yPodOffset.set(-5.628032534141241);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Library defaults are a starting point; replace with Foresight AutoTune output.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.24301486388558605);
                Controller secondaryTranslationalForward = Controller.proportional(0.08978749872896895);
                Controller primaryTranslationalLateral = Controller.proportional(0.32875651018022634);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1214667459758545);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01771882926448662));
                c.brake.set(Controller.proportionalFeedforward(0.015061004874813625));

                c.headingFeedback.set(Controller.proportional(3.535195097256469));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04514813316450815, 0.005222825901884042));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10728986384950936, 0.06037322227576365));
                c.quadraticBrakeCoefficients.set(Matrix.diag(5.236601477698931E-4, 0.001457170146774007));

                c.maxAchievableForwardVelocity.set(50.00);
                c.maxAchievableStrafeVelocity.set(40.00);
                c.naturalForwardDeceleration.set(32.76152418827871);
                c.naturalStrafeDeceleration.set(62.76051571763347);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
