package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@Disabled
@TeleOp(name = "Motor")
public class MotorTeleOp extends OpMode {

    private GamepadEx player1;
    private DcMotorEx shooter;

    private static final int TICKS_PER_REV = 537; // Example: goBILDA 312 RPM Yellow Jacket
    private double target = 1000;   // starting RPM
    private double increment = 50;  // increment/decrement per button press

    @Override
    public void init(){
        player1 = new GamepadEx(gamepad1);
        shooter = hardwareMap.get(DcMotorEx.class, "motor1");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        player1.readButtons();
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            target += increment;
        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            target -= increment;
        }

        // Convert RPM to ticks per second
        double ticksPerSecond = (target / 60.0) * TICKS_PER_REV;
        shooter.setVelocity(ticksPerSecond);

        telemetry.addData("Target RPM", target);
        telemetry.addData("Actual RPM", shooter.getVelocity() * 60.0 / TICKS_PER_REV);
        telemetry.addData("Target TPS", ticksPerSecond);
        telemetry.addData("Actual TPS", shooter.getVelocity());
        telemetry.update();
    }

}
