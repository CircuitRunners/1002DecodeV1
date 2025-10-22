package org.firstinspires.ftc.teamcode.V1.Config.util;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;

/**
 * HeadingAutoAligner (PID Version)
 * ------------------
 * Computes rotation power to face a fixed field coordinate using a PID controller.
 *
 * - Tunable live in panels
 * - Optionally adjustable during TeleOp via button input.
 *
 * Math:
 *   1. worldAngleToTarget = atan2(targetY - robotY, targetX - robotX)
 *   2. headingError = normalizeAngle(worldAngleToTarget - robotHeading)
 *   3. rotationPower = kP * error + kI * sum(error) + kD * (error - lastError)
 *
 * Tuning tips:
 *   - Start with only kP (0.3–1.0)
 *   - Add small kD (0.02–0.05) if oscillating
 *   - Add small kI (0.0005–0.002) if slow to finish alignment
 */
@Configurable
public class HeadingAutoAligner {

    // Tunable PID constants
    public static double kP = 0.6;
    public static double kI = 0.0;
    public static double kD = 0.02;

    // Internal PID memory
    private double targetX;
    private double targetY;
    private double lastError = 0.0;
    private double integral = 0.0;

    // For stability tuning
    public static double integralLimit = 1.0;

    public HeadingAutoAligner(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    public void setTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /**
     * @param robotX Robot X coordinate (same units as target)
     * @param robotY Robot Y coordinate
     * @param robotHeading Robot heading (radians)
     * @return rotation power (-1.0 to 1.0)
     */
    public double getRotationPower(double robotX, double robotY, double robotHeading) {
        double worldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
        double headingError = normalizeAngle(worldAngleToTarget - robotHeading);

        // PID terms
        integral += headingError;
        integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

        double derivative = headingError - lastError;
        lastError = headingError;

        double output = (kP * headingError) + (kI * integral) + (kD * derivative);
        output = Math.max(-1.0, Math.min(1.0, output));

        return output;
    }

    public void reset() {
        lastError = 0;
        integral = 0;
    }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }

    private double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2*Math.PI);
        if (angle < 0) {
            return angle + 2*Math.PI;
        }
        return angle;
    }
}