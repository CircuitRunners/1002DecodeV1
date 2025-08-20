package org.firstinspires.ftc.teamcode.MemDevDecode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp(name = "Chipi Rebuld TeleOp", group = "Test")
public class TankDriveTeleOp extends OpMode {

    private GamepadEx player1;
    TankBot drive = new TankBot();

    public double speedMultiply = 1;


    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        drive.init(hardwareMap);


        telemetry.addLine("Ready!");
        telemetry.update();
    }


    @Override
    public void loop() {


        // Update the speed multiplier based on the D-pad
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            speedMultiply = 1;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            speedMultiply = 0.50;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            speedMultiply = 0.75;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            speedMultiply = 0.25;
        }

        drive();
        telemetry();
    }


    public void drive() {
        double leftSidePower = Range.clip(player1.getLeftY() * speedMultiply, -1, 1);
        double rightSidePower = Range.clip(player1.getRightY() * speedMultiply, -1, 1);
        drive.tankDrive(leftSidePower, rightSidePower);

    }

    public void telemetry(){
        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());
        telemetry.addData("speed multipy value:",speedMultiply);

        telemetry.update();

    }



}
