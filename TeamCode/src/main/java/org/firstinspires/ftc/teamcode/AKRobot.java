package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.*;

public class AKRobot extends Robot {
    public final MecanumDriveProvider mecanumDrive;
    public final Arm arm;
    public final Slide slide;
    //public final Claw claw;
    //public final Wrist wrist;
    //public final Lift lift;

    private final GamepadEx player1Gamepad;
    private final GamepadEx player2Gamepad;

    public AKRobot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.player1Gamepad = new GamepadEx(gamepad1);
        this.player2Gamepad = new GamepadEx(gamepad2);

        this.mecanumDrive = new MecanumDriveProvider(hardwareMap);
        this.arm = new Arm(hardwareMap);
        this.slide = new Slide(hardwareMap);
        //  this.wrist = new Wrist(hardwareMap);
        // this.claw = new Claw(hardwareMap);
        //  this.lift = new Lift(hardwareMap);

        register(mecanumDrive, arm);
    }

    public void initializeCommandOpMode() {
        mecanumDrive.setDefaultCommand(
                mecanumDrive.driveCommand(
                        player1Gamepad::getLeftX,
                        player1Gamepad::getLeftY,
                        player1Gamepad::getRightX
                )
        );

        // set to max power while held, set to 0 when stopped
        player1Gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new StartEndCommand(() -> {
                    arm.getMotor().setRunMode(Motor.RunMode.RawPower);
                    arm.getMotor().set(1);
                }, () -> {
                    arm.getMotor().set(0);
                }));

        // assuming .2 is enough holding power for now
        player1Gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(() -> {
                    arm.getMotor().setRunMode(Motor.RunMode.RawPower);
                    arm.getMotor().set(0.2);
                });

        player1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(slide.extend());

        player1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(slide.retract());
    }
}
