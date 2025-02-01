package org.firstinspires.ftc.teamcode.debug;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpModeConstants;
import org.firstinspires.ftc.teamcode.components.Slide;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@TeleOp(name = "LinearSlideDebugOpMode", group = OpModeConstants.DEBUG_OPMODE)
public class LinearSlideDebugOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        Slide slide = new Slide(hardwareMap);

//        slide.getLeftServo().setRange(0,1000, AngleUnit.DEGREES);
//        slide.getRightServo().setRange(0,1000, AngleUnit.DEGREES);

 //       slide.getLeftServo().setInverted(false);

        GamepadEx gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new InstantCommand(() -> {
                    telemetry.addData("Left servo pos",  slide.getLeftServo().getPosition());
                    telemetry.addData("Right servo pos", slide.getRightServo().getPosition());
                    telemetry.addData("Left servo angle", slide.getLeftServo().getAngle());
                    telemetry.addData("Right servo angle", slide.getRightServo().getAngle());
                    telemetry.update();
                }));

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new InstantCommand(() -> slide.getLeftServo().rotateByAngle(1)));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand(() -> slide.getRightServo().rotateByAngle(1)));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(slide.moveBackward());

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(slide.moveForward());

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new InstantCommand(() -> slide.getLeftServo().turnToAngle(0)), new InstantCommand(() -> slide.getLeftServo().turnToAngle(350)));

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new InstantCommand(() -> slide.getRightServo().turnToAngle(640)), new InstantCommand(() -> slide.getRightServo().turnToAngle(0)));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(slide.retract(), slide.extend());
    }
}
