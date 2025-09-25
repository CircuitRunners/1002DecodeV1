package org.firstinspires.ftc.teamcode.V1.Config.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Poses {

    public enum Alliance { RED, BLUE }

    public static class AlliancePose {
        private final Pose redPose;
        private final Pose bluePose;

        // Explicit values for Red and Blue
        public AlliancePose(Pose redPose, Pose bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        // Mirror convenience: Blue is computed automatically
        public static AlliancePose mirror(Pose redPose) {
            return new AlliancePose(redPose, redPose.mirror());
        }

        public Pose get(Alliance alliance) {
            return (alliance == Alliance.RED) ? redPose : bluePose;
        }
    }

    // 🔹 Alliance selection (same as before)
    public static Alliance currentAlliance = Alliance.RED;
    private static Gamepad previousGamepad = new Gamepad();

    public static Pose get(AlliancePose p) { return p.get(currentAlliance); }

    public static void updateAlliance(Gamepad g, Telemetry telemetry) {
        if (g.dpad_up && !previousGamepad.dpad_up) currentAlliance = Alliance.RED;
        else if (g.dpad_down && !previousGamepad.dpad_down) currentAlliance = Alliance.BLUE;
        previousGamepad.copy(g);

        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addData("Current Alliance", currentAlliance);
        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
        telemetry.update();
    }

    // =======================
    // Example poses
    // =======================

    // ️Mirrored point (Blue auto-computed)
    public static final AlliancePose startPose = AlliancePose.mirror(
            new Pose(7.3285, 65.83, 0)
    );

    //  Non-mirrored point (explicit different coords)
    public static final AlliancePose preloadPose = new AlliancePose(
            new Pose(42, 65.83, 0),     // Red
            new Pose(-40, 60, Math.toRadians(10))  // Blue custom
    );

    //  Another mirrored one
    public static final AlliancePose intakePose = AlliancePose.mirror(
            new Pose(20, 36, Math.toRadians(90))
    );
}