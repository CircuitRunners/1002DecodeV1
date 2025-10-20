package org.firstinspires.ftc.teamcode.V1.Config.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
    public class Intake {

        private Telemetry telemetry;
        private DcMotorEx intake;
        private CRServo servoIntake;

        // Configurable constants
        public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket
        public static double targetRPM = 0;  // default target speed
        public static double servoPower = 0.0;
        private boolean reverseServo = false;

        public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;

            intake = hardwareMap.get(DcMotorEx.class, "intake");
            servoIntake = hardwareMap.get(CRServo.class, "feederWheel");

            intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        // --- Basic control methods ---

        public void setTargetRPM(double rpm) {
            targetRPM = rpm;
            double ticksPerSecond = rpmToTicksPerSecond(rpm);
            intake.setVelocity(ticksPerSecond);
        }

        public void stopIntake() {
            targetRPM = 0;
            intake.setVelocity(0);
        }

        public void setServoPower(double power) {
            servoPower = power;
            servoIntake.set(servoPower);

        }

        public void toggleServoDirection() {
            reverseServo = !reverseServo;
            servoIntake.setInverted(reverseServo);
        }

        public void incrementRPM(double increment) {
            setTargetRPM(targetRPM + increment);
        }

        public void decrementRPM(double increment) {
            setTargetRPM(targetRPM - increment);
        }

        // --- Periodic update (optional) ---
        public void update() {
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", getCurrentRPM());
            telemetry.addData("Servo Power", servoPower);
            telemetry.update();
        }

        // --- Utility methods ---
        private double rpmToTicksPerSecond(double rpm) {
            return (rpm / 60.0) * TICKS_PER_REV;
        }

        public double getCurrentRPM() {
            return intake.getVelocity() * 60.0 / TICKS_PER_REV;
        }

        public double getCurrentVelocity(){
            return intake.getVelocity();
        }
        public double getCurrentTargetRPM(){
            return targetRPM;
        }

        public double getCurrentTargetVelocity(){
            return rpmToTicksPerSecond(targetRPM);
        }
        public double getServoPower(){
            return servoPower;
        }



    }

