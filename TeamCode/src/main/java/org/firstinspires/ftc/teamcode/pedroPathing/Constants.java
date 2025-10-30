package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
           .mass(9.798) //21.6 lbs
           .forwardZeroPowerAcceleration(-42.101)
            .lateralZeroPowerAcceleration(-79.605)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.001,0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013, 0, 0.0001, 0.6, 0.00))
            .centripetalScaling(0.0005)

            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.7, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)


                .build();
    }


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.903)
            .yVelocity(68.5167)
            .useBrakeModeInTeleOp(true)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.5)  //x pod in terms of gobuilda
            .strafePodX(2.5) // y pod in terms of gobuilda
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

}
