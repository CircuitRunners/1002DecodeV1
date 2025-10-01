package org.firstinspires.ftc.teamcode.Testers;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Limelight Relocalize")
public class PedroLimelightRelocalize extends OpMode {

    private Limelight3A limelight;
    private Follower follower;

    // scale factors
    private static final double METERS_TO_INCH = 39.37;
    private static final double INCH_TO_PEDRO  = 1.0 / 0.5; // 1 pedro unit = 0.5 in

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower  = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(72, 72, 0, PedroCoordinates.INSTANCE));
        telemetry.addLine("Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            // --- Limelight meters -> inches -> FTC Pedro units ---
            double xFTC = (result.getBotpose_MT2().getPosition().x * METERS_TO_INCH) * INCH_TO_PEDRO;
            double yFTC = (result.getBotpose_MT2().getPosition().y * METERS_TO_INCH) * INCH_TO_PEDRO;
            double headingRad = Math.toRadians(result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.RADIANS));

            // --- Build Pose in FTC coordinates, then convert to Pedro bottom-left ---
            Pose pedroPose = new Pose(xFTC, yFTC, headingRad, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            follower.setPose(pedroPose);

            telemetry.addData("Limelight FTC  (center-origin Pedro)", "(%.2f , %.2f)", xFTC, yFTC);
            telemetry.addData("Converted Pedro (bottom-left)", "(%.2f , %.2f)",
                    pedroPose.getX(), pedroPose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pedroPose.getHeading()));
        }

        telemetry.update();
    }
}
