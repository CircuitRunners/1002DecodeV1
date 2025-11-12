package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;

@TeleOp
public class DistanceSensorTest  extends OpMode {


    private Intake intake;
    @Override
    public void init(){
        intake = new Intake(hardwareMap, telemetry);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {


        // Convert RPM to ticks per second

//        telemetry.addData("Target RPM", target);
//        telemetry.addData("Intake RPM", intake.getVelocity() * 60.0 / TICKS_PER_REV);
//
//        telemetry.addData("Target TPS", ticksPerSecond);
//        telemetry.addData("Intake TPS", intake.getVelocity());
//
//        telemetry.addData("Servo Power", servoPower); // servoIntake.getPower()
        telemetry.addData("Distance", intake.getDistanceMM());

        telemetry.update();
    }
}
