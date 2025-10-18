package org.firstinspires.ftc.teamcode.MemDevDecode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MemDevDecode.Config.MecanumDrive;

@TeleOp(name = "Mecanum -Robot Centric")
public class MecanumRobotCentricTeleOp extends OpMode {
    private MecanumDrive drive;
    private GamepadEx player1;


    @Override
    public void init(){
        telemetry.addLine("Initializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);


        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        double forward = player1.getLeftY();
        double strafe  =  player1.getLeftX();
        double rotate  =  player1.getRightX();

        /** Send inputs to drive class using method created in Mecanum Drive Class */
        drive.drive(forward, strafe, rotate);

        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());

        telemetry.update();
    }
}
