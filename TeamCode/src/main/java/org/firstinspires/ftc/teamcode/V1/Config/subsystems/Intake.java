package org.firstinspires.ftc.teamcode.V1.Config.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
    public class Intake {

        private Telemetry telemetry;
        private DcMotorEx intake;
        private CRServo servoIntake;
        private DistanceSensor distanceSensor;

        public static double motorPower = 0;

        // Configurable constants
        public static final int TICKS_PER_REV = 537; // goBILDA 312 RPM Yellow Jacket
        public static double targetRPM = 0;  // default target speed
        public static double servoPower = 0.0;
        private boolean reverseServo = false;

        public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;

            intake = hardwareMap.get(DcMotorEx.class, "intake");
            servoIntake = hardwareMap.get(CRServo.class, "feederWheel");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


            servoIntake.setDirection(DcMotorSimple.Direction.REVERSE);

            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
           // intake.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        // --- Basic control methods ---

//        public void setTargetRPM(double rpm) {
//            targetRPM = rpm;
//            double ticksPerSecond = rpmToTicksPerSecond(rpm);
//            intake.setVelocity(ticksPerSecond);
//        }

//        public void stopIntake() {
//            targetRPM = 0;
//            intake.setVelocity(0);
//        }

        public void setServoPower(double power) {
            servoPower = power;
            servoIntake.setPower(servoPower);

        }

//        public void incrementRPM(double increment) {
//            setTargetRPM(targetRPM + increment);
//        }
//
//        public void decrementRPM(double increment) {
//            setTargetRPM(targetRPM - increment);
//        }

        public void intakeIn(){
            intake.setPower(1);
            motorPower = 1;
        }

        public void intakeInDistance(){
            intake.setPower(1);
            motorPower = 1;
            if (distanceSensor.getDistance(DistanceUnit.MM) <= 60){
               servoIntake.setPower(0);
            }
            else if (distanceSensor.getDistance(DistanceUnit.MM )>60){
                servoIntake.setPower(1);
            }
        }

        public void intakeOut(){
            intake.setPower(-1);
            motorPower = -1;
        }

        public void fullIntakeOut(){
            intake.setPower(-1);
            motorPower = -1;
            servoIntake.setPower(-1);
        }
        public void intakeOutDistance(){
        intake.setPower(-1);
        motorPower = -1;
        if (distanceSensor.getDistance(DistanceUnit.MM) <= 60){
            servoIntake.setPower(1);
        }
        else if (distanceSensor.getDistance(DistanceUnit.MM )>60){
            servoIntake.setPower(0);
        }
    }
        public void intakeRetainBalls(){
            intake.setPower(0.5);
            motorPower = 0.5;
        }

        public void fullIntakeIdle(){
            intake.setPower(0);
            servoIntake.setPower(0);
            motorPower = 0;
        }

        public void intakeMotorIdle(){
            intake.setPower(0);
            motorPower = 0;
        }
        public void intakeServoIdle(){
           servoIntake.setPower(0);
        }

        public double getDistanceMM(){
           return distanceSensor.getDistance(DistanceUnit.MM);
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
    public double getPower(){
        return motorPower;
    }




    }

