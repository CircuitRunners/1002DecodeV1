package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
@TeleOp
public class ShooterTest extends OpMode {
    public DcMotorEx shooter;
    public GamepadEx player1;
    double LAUNCHER_TARGET_VELOCITY = 1633; //velocity in ticks per second; 28 ticks per revolution
    final double STOP_SPEED = 0.0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        player1 = new GamepadEx(gamepad1);
    }

    public void loop() {

        player1.readButtons();

        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            shooter.setVelocity((STOP_SPEED));
        }
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            LAUNCHER_TARGET_VELOCITY += 100;
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            LAUNCHER_TARGET_VELOCITY -= 100;
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            LAUNCHER_TARGET_VELOCITY += 10;
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            LAUNCHER_TARGET_VELOCITY -= 10;
        }

        telemetry.addData("Target Speed", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Motor Speed", shooter.getVelocity());
        telemetry.update();
    }
}

