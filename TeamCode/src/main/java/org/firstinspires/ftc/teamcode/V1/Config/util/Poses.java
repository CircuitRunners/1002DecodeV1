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

//        telemetry.addLine("--- Alliance Selector ---");
//        telemetry.addData("Current Alliance", currentAlliance);
//        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
//        telemetry.update();
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


    /**
     * Common Terms:
     * Line 1 - Closest Line To Goal
     * Line 3- Closest line to human player side
     * Line 4 - Actually in human player zone
     */
    // ===============
    // Goal Side Poses (Blue given, red auto-computed
    //================
//    public static final AlliancePose startPoseGoalSide = new AlliancePose(
//            new Pose(111-2, 137+10, Math.toRadians(180)),
//            new Pose(32, 137, Math.toRadians(0))
//    );
//
//    public static final AlliancePose shootPositionGoalSide = new AlliancePose(
//            new Pose(99+4, 116-4, Math.toRadians(45)),
//            new Pose(44, 116, Math.toRadians(128))
//    );
//
//    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
//            new Pose(99+4, 116-4, Math.toRadians(45)),
//            new Pose(44, 116, Math.toRadians(140))
//    );
//
//    public static final AlliancePose lineupLine1 = new AlliancePose(
//            new Pose(89, 92, Math.toRadians(0)),
//            new Pose(54, 92, Math.toRadians(180))
//    );
//    public static final AlliancePose pickupLine1 = new AlliancePose(
//            new Pose(117, 92, Math.toRadians(0)),
//            new Pose(28, 92, Math.toRadians(180))
//    );
//    public static final AlliancePose lineupLine2 = new AlliancePose(
//            new Pose(89, 65, Math.toRadians(0)),
//            new Pose(54, 65, Math.toRadians(180))
//    );
//    public static final AlliancePose pickupLine2 = new AlliancePose(
//            new Pose(123, 65, Math.toRadians(0)),
//            new Pose(20, 65, Math.toRadians(180))
//    );
//    public static final AlliancePose line2ControlPoint = new AlliancePose(
//            new Pose(96, 67, Math.toRadians(0)),
//            new Pose(47, 67, Math.toRadians(180))
//    );
//    public static final AlliancePose lineupLine3 = new AlliancePose(
//            new Pose(91, 38, Math.toRadians(0)),
//            new Pose(52, 38, Math.toRadians(180))
//    );
//    public static final AlliancePose pickupLine3 = new AlliancePose(
//            new Pose(123, 38, Math.toRadians(0)),
//            new Pose(20, 38, Math.toRadians(180))
//    );
//    public static final AlliancePose lineupAtGate = new AlliancePose(
//            new Pose(108, 78.5+5, Math.toRadians(270)),
//            new Pose(27, 78.5, Math.toRadians(90))
//    );

    public static final AlliancePose startPoseGoalSide = new AlliancePose(
            new Pose(112.0, 135.5, Math.toRadians(0)),      // Red: X=144-32=112, Theta=180-180=0
            new Pose(32, 135.5, Math.toRadians(180))        // Blue
    );

    // SHOOT_POSITION_GOAL_SIDE
    public static final AlliancePose shootPositionGoalSide = new AlliancePose(
            new Pose(84.5, 84.0, Math.toRadians(45.0)),     // Red: X=144-59.5=84.5, Theta=180-135=45
            new Pose(59.5, 84, Math.toRadians(135))         // Blue
    );

    // SHOOT_POSITION_GOAL_SIDE_2
    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
            new Pose(105.5, 105.5, Math.toRadians(38.5)),     // Red: X=144-48=96, Theta=180-132.5=47.5
            new Pose(38.5, 105.5, Math.toRadians(130.5))       // Blue
    );

    // LINEUP_LINE_1
    public static final AlliancePose lineupLine1 = new AlliancePose(
            new Pose(90.0, 92.0, Math.toRadians(0)),        // Red: X=144-54=90, Theta=180-180=0
            new Pose(54, 92, Math.toRadians(180))           // Blue
    );

    // CONTROL_POINT_LINE_1_FOR_SHOOT_POSE_2
    public static final AlliancePose controlPointLine1ForShootPose2 = new AlliancePose(
            new Pose(99.0, 82.5, Math.toRadians(0)),
            new Pose(51, 82.5, Math.toRadians(180))
    );

    // PICKUP_LINE_1
    public static final AlliancePose pickupLine1 = new AlliancePose(
            new Pose(126, 80.5, Math.toRadians(0)),
            new Pose(13.5, 84, Math.toRadians(180))
    );

    // LINEUP_LINE_2
    public static final AlliancePose lineupLine2 = new AlliancePose(
            new Pose(90.0, 65.0, Math.toRadians(0)),
            new Pose(54, 65, Math.toRadians(180))
    );

    // PICKUP_LINE_2
    public static final AlliancePose pickupLine2 = new AlliancePose(
            new Pose(136, 55, Math.toRadians(0)),
            new Pose(6.8, 55, Math.toRadians(180))
    );

    // LINE_2_CONTROL_POINT
    public static final AlliancePose line2ControlPoint = new AlliancePose(
            new Pose(82.0, 53.0, Math.toRadians(0)),
            new Pose(62, 53, Math.toRadians(180))
    );

    // LINEUP_LINE_3
    public static final AlliancePose lineupLine3 = new AlliancePose(
            new Pose(92.0, 38.0, Math.toRadians(0)),
            new Pose(52, 38, Math.toRadians(180))
    );

    // PICKUP_LINE_3
    public static final AlliancePose pickupLine3 = new AlliancePose(
            new Pose(136, 36.5, Math.toRadians(0)),
            new Pose(6.8, 36.5, Math.toRadians(180))
    );

    // LINE_3_CONTROL_POINT
    public static final AlliancePose Line3ControlPoint = new AlliancePose(
            new Pose(81.0, 29.5, Math.toRadians(0)),
            new Pose(63, 29.5, Math.toRadians(180))
    );

    // LINEUP_AT_GATE
    public static final AlliancePose lineupAtGate = new AlliancePose(
            new Pose(119.0, 61.0, Math.toRadians(0)),
            new Pose(21, 73, Math.toRadians(180))
    );

    public static final AlliancePose openGate = new AlliancePose(
            new Pose(127.0, 70.0, Math.toRadians(180)),
            new Pose(16.5, 70, Math.toRadians(0))
    );

    // PICKUP_LINE_4
    public static final AlliancePose pickupLine4 = new AlliancePose(
            new Pose(136, 10, Math.toRadians(270)),
            new Pose(0.5, 10, Math.toRadians(270))
    );

    // LINE_4_CONTROL_POINT
    public static final AlliancePose Line4ControlPointClose = new AlliancePose(
            new Pose(138.0, 67.0, Math.toRadians(0)),
            new Pose(1, 67, Math.toRadians(180))
    );

    public static final AlliancePose Line4ControlPointFar1 = new AlliancePose(
            new Pose(97, 28, Math.toRadians(0)),
            new Pose(47, 28, Math.toRadians(180))
    );

    public static final AlliancePose Line4ControlPointFar2 = new AlliancePose(
            new Pose(142, 55, Math.toRadians(0)),
            new Pose(1, 55, Math.toRadians(180))
    );

    public static final AlliancePose Line4ControlPointFarGoBack = new AlliancePose(
            new Pose(98, 1, Math.toRadians(0)),
            new Pose(46, 1, Math.toRadians(180))
    );

    // PICKUP_LINE_1_TO_GATE_CONTROL_POINT
    public static final AlliancePose pickupLine1ToGateControlPoint = new AlliancePose(
            new Pose(103.0, 76.5, Math.toRadians(180)),
            new Pose(41, 76.5, Math.toRadians(0))
    );

    // =============== Far Side Poses (explicit red/blue) ===============

    public static final AlliancePose startPoseFar = new AlliancePose(
            new Pose(88.000, 8.2, Math.toRadians(90)),    // Red: (88, 8, 90)
            new Pose(56.000, 8.2, Math.toRadians(90))     // Blue: (144 - 88, 8, 180 - 90)
    );

    public static final AlliancePose shootPoseFar = new AlliancePose(
            new Pose(83.000, 18.000, Math.toRadians(60)),   // Red: (144-61, 18, 180-120)
            new Pose(61.000, 18.000, Math.toRadians(120))   // Blue: (61, 18, 120)
    );

    // --- Path 2: Bezier Curve ---
    public static final AlliancePose endByHumanPlayer = new AlliancePose(
            new Pose(108.000, 12.5, 0),   // Red: (144-72, 36, 0)
            new Pose(36, 12.5, 180)     // Blue: (72, 36, 0)
    );


    // TELEOP
    public static final AlliancePose teleopDefaultPose = new AlliancePose(
            new Pose(72, 72, Math.toRadians(270)),
            new Pose(72, 72, Math.toRadians(90))
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