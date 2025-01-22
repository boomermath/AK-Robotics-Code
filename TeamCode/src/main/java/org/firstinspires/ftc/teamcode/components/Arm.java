package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.components.subsystem.Targetable;

public class Arm implements Targetable {
    private final MotorGroup armMotors;
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0,0,0,0);

    private final double DEFAULT_ARM_SPEED = 0.5;

    public Arm(HardwareMap hardwareMap) {
        MotorEx leftArmMotor = new MotorEx(hardwareMap, "left_arm");
        MotorEx rightArmMotor = new MotorEx(hardwareMap, "right_arm");

        armMotors = new MotorGroup(leftArmMotor, rightArmMotor);

        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setRunMode(Motor.RunMode.PositionControl);
    }

    public Command moveArm(double speed) {
        return new StartEndCommand(() -> {
            armMotors.setRunMode(Motor.RunMode.VelocityControl);
            armMotors.set(speed);
        }, () -> armMotors.set(armFeedforward.calculate(armMotors.getCurrentPosition(), 0, 0)), this);
    }

    @Override
    public Motor getMotor() {
        return armMotors;
    }

    @Override
    public void changePosition() {
        armMotors.set(armFeedforward.calculate(armMotors.getCurrentPosition(), armMotors.getVelocity()));
    }
}
