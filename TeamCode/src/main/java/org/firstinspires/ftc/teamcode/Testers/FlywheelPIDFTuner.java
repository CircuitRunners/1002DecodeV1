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

@Configurable
@TeleOp
public class FlywheelPIDFTuner extends OpMode {





    /**
     * =============================  FLYWHEEL PIDF TUNER  =============================
     *
     * PURPOSE:
     *      Live-tune a velocity PIDF loop for an FTC flywheel using FTC Dashboard.
     *      The flywheel runs in RUN_WITHOUT_ENCODER, we calculate velocity manually,
     *      and a PIDFController outputs motor power.
     *
     * DASHBOARD SLIDERS YOU CAN TUNE LIVE:
     *      kP, kI, kD, kF       -> PIDF gains
     *      targetVelocity       -> desired speed (encoder ticks/sec)
     *      maxPower             -> power clamp for safety
     *
     * HOW TO USE:
     *  1. Connect FTC Dashboard (phone or laptop) to the RC phone's network
     *     and open http://192.168.43.1:8080 (or your Control Hub IP).
     *  2. Run this OpMode.
     *  3. Start with kP=kI=kD = 0.0.  Set kF ≈ 1.0 / (maxTicksPerSec).
     *     - Find maxTicksPerSec by running the motor at full power once and reading
     *       "Measured Vel" in telemetry.
     *  4. With only kF set, verify the flywheel holds roughly the commanded velocity.
     *  5. Increase kP until the wheel reaches target quickly but without sustained
     *     oscillation.
     *  6. Add small kD to damp overshoot.
     *  7. Add tiny kI only if you still see steady-state error after P+D+F.
     *  8. Adjust targetVelocity up/down and observe spin-up and recovery when you
     *     feed game elements.
     *
     * NOTES:
     *      • Units are encoder ticks/sec. If you prefer RPM:
     *        targetVelocity = (targetRPM * ticksPerRev) / 60.0
     *      • maxPower clamps motor output. Keep ≤1.0.
     *      • After tuning, copy the gains into your flywheel subsystem.
     * ================================================================================
     */

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
        private double lastTicks = 0;
        private double lastTime = 0;

        @Override
        public void init() {
            // Enable bulk reads for faster loops
            List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


            flywheel = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            pidf = new PIDFController(kP, kI, kD, kF);
            pidf.setSetPoint(targetVelocity);

            lastTime = loopTimer.seconds();
        }

        @Override
        public void loop() {
            // ----- Measure velocity (ticks/sec) -----
            double now = loopTimer.seconds();
            double dt = now - lastTime;
            lastTime = now;

            double currentTicks = flywheel.getCurrentPosition();
            double ticksPerSec = (currentTicks - lastTicks) / dt;
            lastTicks = currentTicks;

            // ----- Update gains and setpoint -----
            pidf.setPIDF(kP, kI, kD, kF);
            pidf.setSetPoint(targetVelocity);

            // ----- Compute output and send to motor -----
            double output = pidf.calculate(ticksPerSec, targetVelocity);
            output = Range.clip(output, 0, maxPower);
            flywheel.setPower(output);

            // ----- Telemetry -----
            telemetry.addData("Target Vel (ticks/sec)", targetVelocity);
            telemetry.addData("Measured Vel (ticks/sec)", ticksPerSec);
            telemetry.addData("Motor Power", output);
            telemetry.addData("Loop dt (ms)", dt * 1000);
            telemetry.update();
        }
    }

