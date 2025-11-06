package org.firstinspires.ftc.teamcode.V1.Config.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController; // <--- Import the PIDFController

@Configurable
public class HeadingAutoAligner {

    // Tunable PID constants
    public static double kP = 0.6;
    public static double kI = 0.005;
    public static double kD = 0.03;
    public static double kF = 0.0; // Feedforward is typically zero for heading control

    // Internal state
    private double targetX;
    private double targetY;

    // The PIDF Controller object
    private final PIDFController pidfController;

    public HeadingAutoAligner(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;

        // Initialize the PIDF Controller
        pidfController = new PIDFController(kP, kI, kD, kF);

        pidfController.setSetPoint(0.0);
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;


        pidfController.reset();
    }

    /**
     * @param robotX Robot X coordinate (same units as target)
     * @param robotY Robot Y coordinate
     * @param robotHeading Robot heading (radians)
     * @return rotation power (-1.0 to 1.0)
     */
    public double getRotationPower(double robotX, double robotY, double robotHeading) {


        double worldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // The *error* (the value we feed to the controller) is the difference between the
        // desired angle and the current robot angle, normalized to the shortest path [-pi, pi].
        double headingError = MathFunctions.normalizeAngle(worldAngleToTarget - robotHeading);


        pidfController.setPIDF(kP, kI, kD, kF);


        double output = pidfController.calculate(headingError);


        output = Range.clip(output, -1.0, 1.0);

        return output;
    }

    public void reset() {
        pidfController.reset();
    }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
}