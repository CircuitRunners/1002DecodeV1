package org.firstinspires.ftc.teamcode.SussyTurret;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.List;

/**
 * ================================
 *  TURRET PIDF TUNER (Dashboard)
 * ================================
 *
 * HOW TO TUNE:
 * 1. Start with kF (feedforward).
 *    - Slowly raise kF until the turret *just holds itself in place*
 *      (doesn't drift, but also doesn’t fight you too hard).
 *    - This compensates for friction/gravity.
 *
 * 2. Tune kP.
 *    - Raise kP until turret starts to snap toward the setpoint quickly.
 *    - Too high? It will overshoot or oscillate → back it down.
 *
 * 3. Tune kD.
 *    - Add D to reduce overshoot and jitter.
 *    - Too much → turret feels sluggish.
 *
 * 4. Tune kI last.
 *    - Helps correct small steady-state errors (not reaching setpoint).
 *    - Keep I small, or you’ll get integral windup and oscillations.
 *
 * 5. Use deadband + maxPower.
 *    - deadband: ignore tiny errors near setpoint (removes jitter).
 *    - maxPower: cap output so turret doesn’t whip too hard.
 *
 * Once tuned, copy these constants into your Turret + Limelight align OpMode.
 */
@Configurable
@TeleOp(name = "Turret PIDF Tuner")
public class TurretPIDFTuner extends OpMode {

    private PIDFController turretPIDF;
    private DcMotorEx turretMotor;

    @Override
    public void init() {
        // Enable bulk read for efficiency
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class))
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        turretPIDF = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );
        turretPIDF.setTolerance(TurretConstants.deadband);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Turret PIDF Tuner Ready (Panels)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Always use latest constants from Panels
        turretPIDF.setPIDF(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );
        turretPIDF.setTolerance(TurretConstants.deadband);
        turretPIDF.setSetPoint(TurretConstants.turretSetPoint);

        int turretPos = turretMotor.getCurrentPosition();
        double output = turretPIDF.calculate(turretPos, TurretConstants.turretSetPoint);

        // Deadband
        if (Math.abs(TurretConstants.turretSetPoint - turretPos) < TurretConstants.deadband)
            output = 0;

        // Clamp
        output = Range.clip(output, -TurretConstants.maxPower, TurretConstants.maxPower);
        turretMotor.setPower(output);

        telemetry.addData("Turret Pos", turretPos);
        telemetry.addData("Setpoint", TurretConstants.turretSetPoint);
        telemetry.addData("Error", TurretConstants.turretSetPoint - turretPos);
        telemetry.addData("Output", output);
        telemetry.addData("kP", TurretConstants.kP);
        telemetry.addData("kI", TurretConstants.kI);
        telemetry.addData("kD", TurretConstants.kD);
        telemetry.addData("kF", TurretConstants.kF);
        telemetry.addData("Deadband", TurretConstants.deadband);
        telemetry.addData("MaxPower", TurretConstants.maxPower);
        telemetry.update();
    }
}