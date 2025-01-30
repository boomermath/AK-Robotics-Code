package org.firstinspires.ftc.teamcode.debug;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModeConstants;

@TeleOp(name = "Drive Debug OpMode", group = OpModeConstants.DEBUG_OPMODE)
public class DriveDebugOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motor_front_left");
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motor_front_right");
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motor_back_left");
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motor_back_right");

        motorFrontLeft.setInverted(true);
        motorBackRight.setInverted(true);
        motorBackLeft.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && gamepad1.dpad_up) {
                motorFrontLeft.set(1);
            } else {
                motorFrontLeft.set(0);
            }

            if (gamepad1.b && gamepad1.dpad_up) {
                motorFrontRight.set(1);
            } else {
                motorFrontRight.set(0);
            }

            if (gamepad1.x && gamepad1.dpad_up) {
                motorBackLeft.set(1);
            } else {
                motorBackLeft.set(0);
            }

            if (gamepad1.y && gamepad1.dpad_up) {
                motorBackRight.set(1);
            } else {
                motorBackRight.set(0);
            }
        }
    }
}
