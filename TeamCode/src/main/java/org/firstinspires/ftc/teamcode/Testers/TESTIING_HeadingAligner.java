package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.math.MathFunctions;

public class TESTIING_HeadingAligner {


    /**
     * Simple PID-based heading auto-aligner.
     *
     * Call getRotationPower() with current robot pose to get rotation correction
     * toward a fixed (targetX, targetY) coordinate.
     */


        private double targetX;
        private double targetY;

        private double lastError = 0.0;
        private double integral = 0.0;

        private double integralLimit = 1.0;

        public TESTIING_HeadingAligner(double targetX, double targetY) {
            this.targetX = targetX;
            this.targetY = targetY;
        }

        public void setTarget(double x, double y) {
            this.targetX = x;
            this.targetY = y;
        }

        public double getRotationPower(double robotX, double robotY, double robotHeading,
                                       double kP, double kI, double kD) {
            double worldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
            double headingError = MathFunctions.normalizeAngle(worldAngleToTarget - robotHeading);

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
    }

