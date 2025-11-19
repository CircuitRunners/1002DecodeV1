package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Simple Circle")
public class Circle extends LinearOpMode {

    private static double RADIUS = 20;
    private static double DIAMETER = 2*RADIUS;

    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        Pose midRight = new Pose(0, 0, 0);
        Pose topRight = new Pose(0, RADIUS, Math.toRadians(45));
        Pose topMid = new Pose(-RADIUS, RADIUS, Math.toRadians(90));
        Pose topLeft = new Pose(-DIAMETER, RADIUS, Math.toRadians(135));
        Pose midLeft = new Pose(-DIAMETER, 0, Math.toRadians(180));
        Pose bottomLeft = new Pose(-DIAMETER, -RADIUS, Math.toRadians(225));
        Pose bottomMid = new Pose(-RADIUS, -RADIUS, Math.toRadians(270));
        Pose bottomRight = new Pose(0, -RADIUS, Math.toRadians(315));

        follower.setStartingPose(midRight);

        PathChain circlepath = follower.pathBuilder()
                .addPath(new BezierCurve(midRight, topRight, topMid))
                .setLinearHeadingInterpolation(midRight.getHeading(), topMid.getHeading())
                .addPath(new BezierCurve(topMid, topLeft, midLeft))
                .setLinearHeadingInterpolation(topMid.getHeading(), midLeft.getHeading())
                .addPath(new BezierCurve(midLeft, bottomLeft, bottomMid))
                .setLinearHeadingInterpolation(midLeft.getHeading(), bottomMid.getHeading())
                .addPath(new BezierCurve(bottomMid, bottomRight, midRight))
                .setLinearHeadingInterpolation(bottomMid.getHeading(), midRight.getHeading())
                .build();

        waitForStart();

        if (opModeIsActive()) {
            follower.followPath(circlepath);
        }

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }






    }

}
