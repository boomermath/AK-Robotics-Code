package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.MecanumDriveProvider;

@TeleOp(name = "Drive OpMode", group = "TeleOp Drive")
public class DriveOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        AKRobot robot = new AKRobot(hardwareMap, gamepad1, gamepad2, telemetry);

        robot.initializeCommandOpMode();
    }
}
