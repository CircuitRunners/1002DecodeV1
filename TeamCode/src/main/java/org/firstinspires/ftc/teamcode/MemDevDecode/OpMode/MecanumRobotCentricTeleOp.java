package org.firstinspires.ftc.teamcode.MemDevDecode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MemDevDecode.Config.MecanumDrive;

@TeleOp(name = "Mecanum -Robot Centric")
public class MecanumRobotCentricTeleOp extends OpMode {
    private MecanumDrive drive;
    private GamepadEx player1;
    private double speedMultiply = 0.0;

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

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            speedMultiply = 0.25;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            speedMultiply = 0.5;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            speedMultiply = 0.75;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))  {
            speedMultiply = 1.0;
        }

        double forward = player1.getLeftY() * speedMultiply;
        double strafe  =  player1.getLeftX() * speedMultiply;
        double rotate  =  player1.getRightX() * speedMultiply;

        /** Send inputs to drive class using method created in Mecanum Drive Class */
        drive.drive(forward, strafe, rotate);

        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());
        telemetry.addData("left x", player1.getLeftX());
        telemetry.addData("left y", player1.getLeftY());
        telemetry.addData("right x", player1.getRightX());


        telemetry.update();
    }
}

