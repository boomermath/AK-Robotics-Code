package org.firstinspires.ftc.teamcode.debug;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModeConstants;
import org.firstinspires.ftc.teamcode.commands.ToTargetCommand;
import org.firstinspires.ftc.teamcode.components.Arm;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "Arm Debug OpMode", group = OpModeConstants.DEBUG_OPMODE)
public class ArmTuningOpMode extends CommandOpMode {

    public ArmFeedforward armFeedForward(double ks, double kcos, double kv, double ka) {
        return new ArmFeedforward(ks, kcos, kv, ka);
    }


    public void asdf() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        arm.getMotor().resetEncoder();
        Map<String, Double> values = new HashMap<>();

        values.put("ks", 0.);
        values.put("kcos", 0.);
        values.put("kv", 0.);
        values.put("ka", 0.);
        values.put("kp", 0.);
        values.put("dist per pulse (mm/tick)", 1.);
        List<String> keys = new ArrayList<>(values.keySet());
        AtomicInteger index = new AtomicInteger();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            String selectedValue = keys.get(index.get());
            telemetry.addData("Encoder value", arm.getMotor().getCurrentPosition());
            telemetry.addData("Gain tuned:", selectedValue);
            telemetry.addData("Gain value:", values.get(selectedValue));
            telemetry.update();
            ArmFeedforward armFeedforward = armFeedForward(values.get("ks"), values.get("kcos"), values.get("kv"), values.get("ka"));

            arm.getMotor().setDistancePerPulse(values.get("dist per pulse (mm/tick)"));

            driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(() -> {
                        values.computeIfPresent(selectedValue, (key, v) -> (v + 0.01));
                        sleep(100);
                    });

            driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(() -> values.computeIfPresent(selectedValue, (key, v) -> (v - 0.01)));

            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(arm.targetTicks(60));

            driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                    .whenPressed(() -> {
                        if (index.get() == 4) {
                            index.set(0);
                        } else {
                            index.incrementAndGet();
                        }
                    });

            CommandScheduler.getInstance().run();
        }
    }

    @Override
    public void initialize() {
        Arm arm = new Arm(hardwareMap);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new StartEndCommand(() -> {
                    arm.getMotor().setRunMode(Motor.RunMode.RawPower);
                    arm.getMotor().set(0.3);
                }, () -> {
                    arm.getMotor().set(0);
                }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ToTargetCommand<>(130, arm));
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    arm.getMotor().setRunMode(Motor.RunMode.RawPower);
                    arm.getMotor().set(1);
                });

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(() -> {
                    telemetry.addData("encoder", arm.getMotor().getCurrentPosition());
                    telemetry.update();
                });

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(() -> {
                    arm.getMotor().resetEncoder();
                });
    }
}
