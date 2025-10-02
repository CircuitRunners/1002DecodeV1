package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class TurretController {
    private double minAngle;   // radians, e.g. -Math.PI/2
    private double maxAngle;   // radians, e.g.  Math.PI/2
    private double turretAngle; // current turret angle (relative to robot base, radians)

    public TurretController(double minAngle, double maxAngle) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.turretAngle = 0;
    }

    /**
     * Update turret to aim at a target point in world coordinates.
     * @param robotX  Robot base X position
     * @param robotY  Robot base Y position
     * @param robotHeading  Robot heading (world orientation, radians)
     * @param targetX  Target X position
     * @param targetY  Target Y position
     * @return Desired turret angle relative to the robot base (clamped to limits)
     */
    public double aimAt(double robotX, double robotY, double robotHeading,
                        double targetX, double targetY) {

        // World angle from robot → target
        double worldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Convert into turret-relative frame (relative to robot’s heading)
        double relativeAngle = MathFunctions.normalizeAngle(worldAngleToTarget - robotHeading);

        // Clamp to turret’s physical range
        turretAngle = clamp(relativeAngle, minAngle, maxAngle);

        return turretAngle;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    // Utility: clamp within limits
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

/* example usage
    TurretController turret = new TurretController(-Math.PI/2, Math.PI/2); // -90° to +90°

    // Suppose robot is at (0,0), facing east (heading = 0 rad).
// Target is at (0, 10), i.e. north.
    double turretAngle = turret.aimAt(follower.getX(), follower.getY(), follower.getHeading(), 0, 10);

// turretAngle will be +90° (π/2 radians), clamped to max
    Telemetry.addData("Turret relative angle: " + Math.toDegrees(turretAngle));


 */
}