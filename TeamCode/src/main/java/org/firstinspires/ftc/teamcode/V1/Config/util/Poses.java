package org.firstinspires.ftc.teamcode.V1.Config.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Poses {

    //  Alliance enum
    public enum Alliance {
        RED,
        BLUE
    }

    //  AlliancePose holds both red and blue values
    public static class AlliancePose {
        private final Pose redPose;
        private final Pose bluePose;

        public AlliancePose(Pose redPose, Pose bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        public Pose get(Alliance alliance) {
            return (alliance == Alliance.RED) ? redPose : bluePose;
        }
    }

    //  Alliance selection (set this before using poses)
    public static Alliance currentAlliance = Alliance.RED;
    private static Gamepad previousGamepad;

    //  Example poses (Red / Blue)
    public static final AlliancePose startPose = new AlliancePose(
            new Pose(7.3285, 65.83, 0),   // Red
            new Pose(-7.3285, 65.83, 0)   // Blue
    );

    public static final AlliancePose sixSpecStartPose = new AlliancePose(
            new Pose(7.3285, 65.83, 90),   // Red
            new Pose(-7.3285, 65.83, 0)   // Blue
    );

    //  Combined push pathchain (example subset)
    public static final AlliancePose spline1Control = new AlliancePose(
            new Pose(23, 37, 180),
            new Pose(-23, 37, 0)
    );



    //  Helper method to fetch the correct pose for the current alliance
    public static Pose get(AlliancePose pose) {
        return pose.get(currentAlliance);
    }

    /**
     * Call this in init_loop() of an auto to allow the driver to change alliance with the D-pad.
     * D-pad up → RED, D-pad down → BLUE
     * Also updates telemetry to show the current alliance and instructions.
     */
    public static void updateAlliance(Gamepad g, Telemetry telemetry) {
        // Edge detection: trigger only when the button is first pressed
        if (g.dpad_up && !previousGamepad.dpad_up) {
            currentAlliance = Alliance.RED;
        } else if (g.dpad_down && !previousGamepad.dpad_down) {
            currentAlliance = Alliance.BLUE;
        }

        // Update previousGamepad state for next loop
        previousGamepad.copy(g);

        //  Telemetry instructions
        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addData("Current Alliance", currentAlliance);
        telemetry.addLine("Press D-pad UP → RED");
        telemetry.addLine("Press D-pad DOWN → BLUE");
        telemetry.update();
    }
}