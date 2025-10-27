package org.firstinspires.ftc.teamcode.V1.Config.util;

import android.text.InputFilter;

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

        // Mirror convenience: Red is computed automatically
        public static AlliancePose mirror(Pose bluePose) {
            return new AlliancePose(bluePose.mirror(), bluePose);
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

    // ️Mirrored point (Red auto-computed)


    //  Non-mirrored point (explicit different coords)
  /*EXAMPLE*/  public static final AlliancePose preloadPose = new AlliancePose(
            new Pose(42, 65.83, 0),     // Red
            new Pose(-40, 60, Math.toRadians(10))  // Blue custom
    );

    // ===============
    // Goal Side Poses (Blue given, red auto-computed
    //================
    public static final AlliancePose startPoseGoalSide = AlliancePose.mirror(
            new Pose(33, 137, Math.toRadians(0))
    );

    //Used multiple times cause shoot from same spot
    public static final AlliancePose shootPositionGoalSide = AlliancePose.mirror(
            new Pose(38, 106.5, Math.toRadians(132.5))
            //new Pose(48, 95.4, Math.toRadians(135))
    );

    public static final AlliancePose lineupLine1 = AlliancePose.mirror(
            new Pose(43, 84, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine1 = AlliancePose.mirror(
            new Pose(15, 84, Math.toRadians(180))
    );
    public static final AlliancePose lineupLine2 = AlliancePose.mirror(
            new Pose(43, 60, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine2 = AlliancePose.mirror(
            new Pose(9, 60, Math.toRadians(180))
    );

    public static final AlliancePose lineupLine3 = AlliancePose.mirror(
            new Pose(43, 35.5, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine3 = AlliancePose.mirror(
            new Pose(9, 60, Math.toRadians(180))
    );
    public static final AlliancePose lineupAtGate = AlliancePose.mirror(
            new Pose(19, 70, Math.toRadians(0))
    );
    public static final AlliancePose openGate = AlliancePose.mirror(
            new Pose(15.2,70,Math.toRadians(0))
    );
    public static final AlliancePose openGateFromStartControlPoint = AlliancePose.mirror(
            new Pose(79,74,Math.toRadians(0))   //NEED to have tv based pose change to not hit first row of balls
    );
    public static final AlliancePose lineupLine2FromGate = AlliancePose.mirror(
            new Pose(44,60,Math.toRadians(180))
    );
    public static final AlliancePose lineupLine2FromGateControlPoint = AlliancePose.mirror(
            new Pose(37,85,Math.toRadians(0))
    );




    // ===============
    // Goal Side Poses (Blue given, red auto-computed
    //================
    public static final AlliancePose startPoseFarSide = AlliancePose.mirror(
            new Pose(54.6, 8.3, 90)
    );
    public static final AlliancePose pathToShootFirstTimeControlPoint1 = AlliancePose.mirror(
            new Pose(59.4, 48.5, 0)
    );
    public static final AlliancePose pathToShootFirstTimeControlPoint2 = AlliancePose.mirror(
            new Pose(71.8, 98.7, 0)
    );
    public static final AlliancePose shootPositionFar = AlliancePose.mirror(
            new Pose(59, 85, 135)
    );
    public static final AlliancePose toHumanPlayerControlPoint1 = AlliancePose.mirror(
            new Pose(69.7, 97.9, 0)
    );
    public static final AlliancePose toHumanPlayerControlPoint2 = AlliancePose.mirror(
            new Pose(74, 16, 0)
    );
    public static final AlliancePose humanPlayerPosition = AlliancePose.mirror(
            new Pose(18, 16, 180)
    );
    public static final AlliancePose backToShootControlPoint1 = AlliancePose.mirror(
            new Pose(74, 16, 0)
    );
    public static final AlliancePose backToShootControlPoint2 = AlliancePose.mirror(
            new Pose(69.7, 97.9, 0)
    );


    //TELEOP STUFF
    public static final AlliancePose teleopDefaultPose = AlliancePose.mirror(
            new Pose(72, 72, 90) // change to your preferred TeleOp "spawn" location
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