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
    final double LAUNCHER_TARGET_VELOCITY = 1633;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        player1 = new GamepadEx(gamepad1);
    }

    public void loop() {

        player1.readButtons();

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }
}

