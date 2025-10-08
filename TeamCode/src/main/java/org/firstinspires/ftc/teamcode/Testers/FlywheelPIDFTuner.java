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
        // --- Manual calculation is replaced by the built-in method ---
        double rPM = flywheel.getVelocity();

        // --- Update gains and setpoint ---
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);

        // --- Compute output and send to motor ---
        // 'ticksPerSec' (the measured velocity) is passed to the custom PIDF controller.
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