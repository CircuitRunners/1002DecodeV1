package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp

public class PedroExampleAprilTag extends OpMode {

    private Limelight3A limelight; //any camera here
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(0,0,0); //Put the target location here

    private static final double METERS_TO_INCH = 39.37;
    private static final double INCH_TO_PEDRO = 1.0 / 0.5; // 1 pedro unit = 0.5 in

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0)); //set your starting pose
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();

        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc

//        if (!following) {
//            follower.followPath(
//                    follower.pathBuilder()
//                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
//                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
//                            .build()
//            );
//        }

        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
        follower.setPose(getRobotPoseFromCamera());

        if (following && !follower.isBusy()) following = false;
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            // --- Limelight meters -> inches -> FTC Pedro units ---
            double xFTC = (result.getBotpose_MT2().getPosition().x * METERS_TO_INCH) * INCH_TO_PEDRO;
            double yFTC = (result.getBotpose_MT2().getPosition().y * METERS_TO_INCH) * INCH_TO_PEDRO;
            double headingRad = Math.toRadians(result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.RADIANS));

            // --- Build Pose in FTC coordinates, then convert to Pedro bottom-left ---
            Pose pedroPose = new Pose(xFTC, yFTC, headingRad, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
            return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
        return follower.getPose();
    }
}

