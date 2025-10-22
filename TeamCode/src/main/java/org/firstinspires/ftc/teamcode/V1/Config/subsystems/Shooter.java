package org.firstinspires.ftc.teamcode.V1.Config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {

    private Telemetry telemetry;
    public static double kP = 0.002;       // proportional gain
    public static double kI = 0.0001;      // integral gain
    public static double kD = 0.0004;      // derivative gain
    public static double kF = 0.0002;      // feedforward ≈ 1 / maxTicksPerSec
    public static double targetVelocity = 1633; // desired speed (ticks/sec)
    public static double maxPower = 1.0;          // safety clamp

    public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket

    // ===== Hardware =====
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    private PIDFController pidf;
    private ElapsedTime loopTimer = new ElapsedTime();


    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry= telemetry;

        shooter1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "motor2");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);


    }


    public void shoot(){
        loopTimer.reset();

        double rPM = shooter1.getVelocity();
        double rPM2 = shooter2.getVelocity();

        double averageRPM = (rPM + rPM2) / 2;

        // --- Update gains and setpoint ---
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);

        // --- Compute output and send to motor ---
        // 'rPM' (the measured velocity) is passed to the custom PIDF controller.
        double output = pidf.calculate(averageRPM, targetVelocity);
        output = Range.clip(output, 0, maxPower);
        shooter1.setPower(output);
        shooter2.setPower(output);

        // --- Telemetry ---
        telemetry.addData("Target Vel (ticks/sec)", targetVelocity);
        telemetry.addData("Measured Vel motor 1 (tick/sec)", rPM);
        telemetry.addData("Measured Vel motor 1 (tick/sec)", rPM2);
        telemetry.addData("Measured Avg Vel (tick/sec)", averageRPM);
        telemetry.addData("RPM?", getCurrentRPM());
        telemetry.addData("Motor Power", output);
        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.update();

        // Reset the timer for the next loop iteration (optional, for loop time tracking)
        loopTimer.reset();
    }

        // --- Basic control methods ---


        public void setPower(double power){
            shooter1.setPower(power);
            shooter2.setPower(power);
        }
        public void setTargetRPM(double rpm) {
            targetVelocity = rpm;
        }

        public void stopShooter() {
            setPower(0);
        }

        public void incrementRPM(double increment) {
            setTargetRPM(targetVelocity+ increment);
        }

        public void decrementRPM(double increment) {
            setTargetRPM(targetVelocity - increment);
        }


        public double getCurrentRPM() {
            return shooter1.getVelocity() * 60.0 / TICKS_PER_REV;
        }
        public double getCurrentVelo(){
            return ((shooter1.getVelocity()) + (shooter2.getVelocity()))/2;
        }




    }
