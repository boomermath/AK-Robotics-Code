package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class MecanumDriveProvider extends SubsystemBase {
    private final MecanumDrive mecanumDrive;

    public MecanumDriveProvider(HardwareMap hardwareMap) {
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motor_front_left");
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motor_front_right");
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motor_back_left");
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motor_back_right");

        motorFrontLeft.setInverted(true);
        motorBackLeft.setInverted(true);

        mecanumDrive = new MecanumDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
    }

    public Command driveCommand(Supplier<Double> strafe, Supplier<Double> forward, Supplier<Double> yaw) {
        return new RunCommand(() -> mecanumDrive.driveRobotCentric(strafe.get(), forward.get(), yaw.get()), this);
    }
}
