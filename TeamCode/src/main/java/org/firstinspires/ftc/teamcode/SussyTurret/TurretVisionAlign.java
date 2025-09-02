package org.firstinspires.ftc.teamcode.SussyTurret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.SussyTurret.TurretPIDFTuner;


// Import the PIDF constants from the tuner


    /**
     * ================================
     *  TURRET VISION ALIGN (Panels)
     * ================================
     *
     * Uses the PIDF constants directly from TurretPIDFTunerPanels.
     * This ensures Vision Align uses whatever you tuned live in the tuner.
     */
    @TeleOp(name = "Turret Vision Align (Panels with Tuner PIDF)")
    public class TurretVisionAlign extends OpMode {

        private PIDFController turretPIDF;
        private DcMotorEx turretMotor;
        private Limelight3A limelight;

        @Override
        public void init() {
            // Bulk caching for efficiency
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class))
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            turretPIDF = new PIDFController(
                    TurretConstants.kP,
                    TurretConstants.kI,
                    TurretConstants.kD,
                    TurretConstants.kF
            );
            turretPIDF.setTolerance(TurretConstants.deadband);
            turretPIDF.setSetPoint(0); // center target

            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();

            telemetry.addLine("Turret Vision Align Ready (Panels)");
            telemetry.update();
        }

        @Override
        public void loop() {
            // Always use the latest PIDF from Tuner
            turretPIDF.setPIDF(
                    TurretConstants.kP,
                    TurretConstants.kI,
                    TurretConstants.kD,
                    TurretConstants.kF
            );
            turretPIDF.setTolerance(TurretConstants.deadband);

            LLResult result = limelight.getLatestResult();
            double tx = 0;
            boolean validTarget = false;

            if (result != null && result.isValid()) {
                tx = result.getTx();
                validTarget = true;
            }

            double output = 0.0;
            if (validTarget) {
                double error = -tx; // invert if needed
                if (Math.abs(error) < TurretConstants.deadband) error = 0.0;
                output = turretPIDF.calculate(error, 0.0);
                output = Range.clip(output, -TurretConstants.maxPower, TurretConstants.maxPower);
            }

            turretMotor.setPower(output);

            telemetry.addData("Target Visible", validTarget);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("Turret Power", output);
            telemetry.addData("kP", TurretConstants.kP);
            telemetry.addData("kI", TurretConstants.kI);
            telemetry.addData("kD", TurretConstants.kD);
            telemetry.addData("kF", TurretConstants.kF);
            telemetry.addData("Deadband", TurretConstants.deadband);
            telemetry.addData("MaxPower", TurretConstants.maxPower);
            telemetry.update();
        }
    }

