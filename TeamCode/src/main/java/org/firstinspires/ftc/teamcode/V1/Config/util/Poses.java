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
    public static final AlliancePose startPoseGoalSide = new AlliancePose(
            new Pose(111-2, 137+10, Math.toRadians(180)),
            new Pose(32, 137, Math.toRadians(0))
    );

    public static final AlliancePose shootPositionGoalSide = new AlliancePose(
            new Pose(99+4, 116-4, Math.toRadians(45)),
            new Pose(44, 116, Math.toRadians(128))
    );

    public static final AlliancePose shootPositionGoalSide2 = new AlliancePose(
            new Pose(99+4, 116-4, Math.toRadians(45)),
            new Pose(44, 116, Math.toRadians(140))
    );

    public static final AlliancePose lineupLine1 = new AlliancePose(
            new Pose(89, 92, Math.toRadians(0)),
            new Pose(54, 92, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine1 = new AlliancePose(
            new Pose(117, 92, Math.toRadians(0)),
            new Pose(28, 92, Math.toRadians(180))
    );
    public static final AlliancePose lineupLine2 = new AlliancePose(
            new Pose(89, 65, Math.toRadians(0)),
            new Pose(54, 65, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine2 = new AlliancePose(
            new Pose(123, 65, Math.toRadians(0)),
            new Pose(20, 65, Math.toRadians(180))
    );
    public static final AlliancePose backToShoot2ControlPoint = new AlliancePose(
            new Pose(96, 67, Math.toRadians(0)),
            new Pose(47, 67, Math.toRadians(180))
    );
    public static final AlliancePose lineupLine3 = new AlliancePose(
            new Pose(91, 38, Math.toRadians(0)),
            new Pose(52, 38, Math.toRadians(180))
    );
    public static final AlliancePose pickupLine3 = new AlliancePose(
            new Pose(123, 38, Math.toRadians(0)),
            new Pose(20, 38, Math.toRadians(180))
    );
    public static final AlliancePose lineupAtGate = new AlliancePose(
            new Pose(108, 78.5+5, Math.toRadians(270)),
            new Pose(27, 78.5, Math.toRadians(90))
    );
    public static final AlliancePose openGate = new AlliancePose(
            new Pose(128.8, 70, Math.toRadians(180)),
            new Pose(15.2, 70, Math.toRadians(0))
    );
    public static final AlliancePose openGateFromStartControlPoint = new AlliancePose(
            new Pose(65, 74, Math.toRadians(180)),
            new Pose(79, 74, Math.toRadians(0))
    );
    public static final AlliancePose lineupLine2FromGate = new AlliancePose(
            new Pose(100, 60, Math.toRadians(0)),
            new Pose(44, 60, Math.toRadians(180))
    );
    public static final AlliancePose lineupLine2FromGateControlPoint = new AlliancePose(
            new Pose(107, 85, Math.toRadians(180)),
            new Pose(37, 85, Math.toRadians(0))
    );

    // =============== Far Side Poses (explicit red/blue) ===============

    public static final AlliancePose startPoseFar = new AlliancePose(
            new Pose(88.000, 8.000, Math.toRadians(90)),    // Red: (144-56, 8, 180-90)
            new Pose(56.000, 8.000, Math.toRadians(90))     // Blue: (56, 8, 90)
    );

    public static final AlliancePose shootPoseFar = new AlliancePose(
            new Pose(83.000, 18.000, Math.toRadians(60)),   // Red: (144-61, 18, 180-120)
            new Pose(61.000, 18.000, Math.toRadians(120))   // Blue: (61, 18, 120)
    );

    // --- Path 2: Bezier Curve ---
    public static final AlliancePose intakeLine1FarControlPoint = new AlliancePose(
            new Pose(72.000, 36.000, 0),   // Red: (144-72, 36, 0)
            new Pose(72.000, 36.000, 0)     // Blue: (72, 36, 0)
    );

    public static final AlliancePose intakeLine1FarEndPoint = new AlliancePose(
            new Pose(124.000, 36.000, Math.toRadians(0)),
            new Pose(20.000, 36.000, Math.toRadians(180))
    );


    public static final AlliancePose intakeLine2FarControlPoint1 = new AlliancePose(
            new Pose(112.000, 28.000, 0),
            new Pose(32.000, 28.000, 0)
    );

    public static final AlliancePose intakeLine2FarControlPoint2 = new AlliancePose(
            new Pose(137.000, 96.000, 0),
            new Pose(7.000, 96.000, 0)
    );

    public static final AlliancePose intakeLine2FarEndPoint = new AlliancePose(
            new Pose(136.000, 8.000, Math.toRadians(270)),
            new Pose(8.000, 8.000, Math.toRadians(270))
    );

    public static final AlliancePose endInHumanPlayer = new AlliancePose(
            new Pose(129.000, 17.000, Math.toRadians(320)),
            new Pose(15.000, 17.000, Math.toRadians(220))
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