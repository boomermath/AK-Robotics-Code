package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.*;

public class AKRobot extends Robot {
    public final MecanumDriveProvider mecanumDrive;
    public final Arm arm;
    public final Slide slide;
    public final Claw claw;
    public final Wrist wrist;

    private final GamepadEx player1Gamepad;
    private final GamepadEx player2Gamepad;

    public AKRobot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.player1Gamepad = new GamepadEx(gamepad1);
        this.player2Gamepad = new GamepadEx(gamepad2);

        this.mecanumDrive = new MecanumDriveProvider(hardwareMap);
        this.arm = new Arm(hardwareMap);
        this.slide = new Slide(hardwareMap);
        this.wrist = new Wrist(hardwareMap);
        this.claw = new Claw(hardwareMap);

        register(mecanumDrive, arm, slide, wrist, claw);
    }

    public void initializeCommandOpMode() {
        mecanumDrive.setDefaultCommand(
                mecanumDrive.driveCommand(
                        player1Gamepad::getLeftX,
                        player1Gamepad::getLeftY,
                        player1Gamepad::getRightX
                )
        );

        player1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(slide.moveSlide(false));

        player1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(slide.moveSlide(true));

        player1Gamepad.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(wrist.rotateBy(5), wrist.rotateBy(-5));

        player1Gamepad.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(claw.open(), claw.close());

        new Trigger(() -> player1Gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0)
                .whenActive(arm.moveArm(-player1Gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        new Trigger(() -> player1Gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
                .whenActive(arm.moveArm(player1Gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }
}
