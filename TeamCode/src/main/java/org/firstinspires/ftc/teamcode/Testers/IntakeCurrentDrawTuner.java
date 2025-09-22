package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@TeleOp
public class IntakeCurrentDrawTuner extends LinearOpMode {




    /**
     * Run this OpMode to measure intake current draw and identify a jam threshold.
     *
     * Controls:
     *   • Left stick Y  -> intake power (-1.0 to 1.0)
     *   • Press and hold at different powers, observe "Current (A)" in telemetry
     *
     * Goal:
     *   • Note the current when running normally.
     *   • Gently block/jam the intake and observe the spike.
     *   • Use that spike as the JAM_CURRENT_THRESHOLD in Intake.java
     */

        // Dashboard-tunable default power if you don't want to use stick
        public static double testPower = 0.5;

        @Override
        public void runOpMode() throws InterruptedException {


            DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {
                // Either use left stick Y or dashboard slider for power
                double stickPower = -gamepad1.left_stick_y;  // up is negative
                double power = (Math.abs(stickPower) > 0.05) ? stickPower : testPower;

                intake.setPower(Range.clip(power, -1.0, 1.0));

                double amps = intake.getCurrent(CurrentUnit.AMPS);

                telemetry.addData("Set Power", power);
                telemetry.addData("Current (A)", amps);
                telemetry.addLine("Jam the intake carefully to observe spike");
                telemetry.update();
            }
        }
    }

