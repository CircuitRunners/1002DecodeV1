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

    // =======================
    // Alliance selection
    // =======================
    private static Alliance currentAlliance = Alliance.RED;
    private static Gamepad previousGamepad = new Gamepad();

    public static Alliance getAlliance() {
        return currentAlliance;
    }
    private static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    public static Pose get(AlliancePose p) {
        return p.get(currentAlliance);
    }

    public static void updateAlliance(Gamepad g, Telemetry telemetry) {
        if (g.dpad_up && !previousGamepad.dpad_up) setAlliance(Alliance.RED);
        else if (g.dpad_down && !previousGamepad.dpad_down) setAlliance(Alliance.BLUE);
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

    public static final AlliancePose teleopDefaultPose = AlliancePose.mirror(
            new Pose(72, 72, 0) // change to your preferred TeleOp "spawn" location
    );

    // =======================
    // Cross-OpMode pose storage
    // =======================

    /**
     * This holds the "last known" robot pose across OpModes.
     * Autonomous should set this at the end, TeleOp should read it on init.
     */
    private static Pose lastPose = null;

    /** Save a pose (e.g. at end of Auto) */
    public static void savePose(Pose pose) {
        lastPose = pose;
    }

    /** Get the correct starting pose: last saved if available, else default startPose */
    public static Pose getStartingPose() {
        return (lastPose != null) ? lastPose : get(teleopDefaultPose);
    }

    /** Clear saved pose (optional, e.g. between practice runs) */
    public static void reset() {
        lastPose = null;
    }
    /* usage:

    at end of autonomous:

    Poses.savePose(follower.getPose());

    at start of teleop:
    follower.setStartingPose(Poses.getStartingPose());
     */
}