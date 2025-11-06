package org.firstinspires.ftc.teamcode.V1.Config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {

    private Telemetry telemetry;
    public static double kP = 0.01;       // proportional gain
    public static double kI = 0.000;      // integral gain
    public static double kD = 0.000;      // derivative gain
    public static double kF = 0.00025;      // feedforward ≈ 1 / maxTicksPerSec
    public static double targetVelocity = 0; // desired speed (ticks/sec) // 1633og, 1110 new for close/auto spot
    private static double maxPower = 1.0;          // safety clamp

    public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket

    // --- Pre-calculated Linear Model ---
    // Slope (m) = (1250 - 1000) / (48 - 12) = 6.9444...
    private static final double VELO_SLOPE = 7.41071;

    private static final double VELO_INTERCEPT = 802.23214;

    private static final double MIMIMUM_VELO =1000;
    private static final double MAXIMUM_VELO = 2000;

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
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidf = new PIDFController(kP, kI, kD, kF);
        //pidf.setPIDF(kP, kI, kD, kF);
        //pidf.setSetPoint(targetVelocity);


    }


    public void shoot(double desiredVelo){


        targetVelocity = desiredVelo;


        double shooter1Velocity = shooter1.getVelocity();
        double shooter2Velocity = shooter2.getVelocity();

        double averageVelo = (shooter1Velocity + shooter2Velocity) / 2;

        // --- Update gains and setpoint ---


        // --- Compute output and send to motor ---
        // 'shooter1Velocity' (the measured velocity) is passed to the custom PIDF controller.
        double output = pidf.calculate(averageVelo, desiredVelo);
        output = Range.clip(output, 0, maxPower);
        shooter1.setPower(output);
        shooter2.setPower(output);

        // --- Telemetry ---
        telemetry.addData("Target Vel (ticks/sec)", targetVelocity);
//        telemetry.addData("Measured Vel motor 1 (tick/sec)", shooter1Velocity);
//        telemetry.addData("Measured Vel motor 1 (tick/sec)", shooter2Velocity);
        telemetry.addData("Measured Avg Vel (tick/sec)", averageVelo);
        telemetry.addData("Shooter Motor Power Set Point: ", output);
        telemetry.addData("At Set Point?", pidf.atSetPoint() ? "Yes" : "Nah gang");
        telemetry.update();

        // Reset the timer for the next loop iteration (optional, for loop time tracking)

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
    public double getTargetVelocity(){
        return targetVelocity;
    }

    public double calculateFlywheelVelocity(double distanceInInches) {
        // Use the simple linear equation: V = m*D + b
        double targetVelocity = (VELO_SLOPE * distanceInInches) + VELO_INTERCEPT;

        // Safety check: ensure velocity doesn't go below the lowest tuned bound
        targetVelocity = Range.clip(targetVelocity, MIMIMUM_VELO, MAXIMUM_VELO);

        return targetVelocity;
    }





}