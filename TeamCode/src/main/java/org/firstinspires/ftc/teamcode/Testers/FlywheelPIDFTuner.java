package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.List;


/**
 * ============================================================
 *                Flywheel PIDF Tuning Instructions
 * ============================================================
 * This OpMode helps you tune your flywheel velocity PIDF loop.
 * Use FTC Dashboard to view and adjust parameters live.
 *
 * STEP 1: Connect Pannels
 *    - Run this OpMode.
 *    - Open the Dashboard.
 *    - Watch telemetry for velocity, target, and power.
 *
 * STEP 2: Tune Feedforward (kF)
 *    - Set kP, kI, kD = 0.
 *    - Increase kF until the flywheel speed roughly reaches your target velocity
 *      (targetVelocity) without oscillation or overshooting.
 *    - The idea: power ≈ kF * targetVelocity → stable near target speed.
 *
 * STEP 3: Tune Proportional (kP)
 *    - Slowly increase kP until the flywheel reacts quickly but starts to oscillate slightly.
 *    - If it overshoots or oscillates too much, reduce kP a bit.
 *
 * STEP 4: Tune Derivative (kD)
 *    - Add a small amount of kD to reduce oscillations and overshoot.
 *    - This smooths out the response.
 *
 * STEP 5: Tune Integral (kI)
 *    - Add a little kI if there’s a consistent steady-state error (flywheel settles below target).
 *    - Be careful: too much kI will cause oscillation or lag.
 *
 * STEP 6: Verify Response
 *    - Try different targetVelocity values.
 *    - Ensure response is smooth, fast, and consistent under load.
 *
 * STEP 7: Save Constants
 *    - Once tuned, record your PIDF constants and hardcode them in your shooter control code.
 *
 *    NOTE - if its good enough with just Kf and Kp then just leave it - no need for Kd and Ki unless absoulutely neccesary
 *    **/
@Configurable
@TeleOp
public class FlywheelPIDFTuner extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.002;       // proportional gain
    public static double kI = 0.0001;      // integral gain
    public static double kD = 0.0004;      // derivative gain
    public static double kF = 0.0002;      // feedforward ≈ 1 / maxTicksPerSec
    public static double targetVelocity = 3000.0; // desired speed (ticks/sec)
    public static double maxPower = 1.0;          // safety clamp

    // ===== Hardware =====
    private DcMotorEx flywheel;
    private PIDFController pidf;
    private ElapsedTime loopTimer = new ElapsedTime();
    // Removed lastTicks and lastTime since they're no longer needed for manual calculation

    @Override
    public void init() {
        // Enable bulk reads for faster loops
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Important: Keep RUN_WITHOUT_ENCODER so the custom PIDF loop controls power.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);
    }

    @Override
    public void loop() {

        double rPM = flywheel.getVelocity();

        // --- Update gains and setpoint ---
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);

        // --- Compute output and send to motor ---
        // 'rPM' (the measured velocity) is passed to the custom PIDF controller.
        double output = pidf.calculate(rPM, targetVelocity);
        output = Range.clip(output, 0, maxPower);
        flywheel.setPower(output);

        // --- Telemetry ---
        telemetry.addData("Target Vel (rotations/minute)", targetVelocity);
        telemetry.addData("Measured Vel (rotations/minute)", rPM);
        telemetry.addData("Motor Power", output);
        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.update();

        // Reset the timer for the next loop iteration (optional, for loop time tracking)
        loopTimer.reset();
    }
}