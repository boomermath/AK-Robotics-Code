package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final MotorGroup armMotors;
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0,0,0,0);

    public Arm(HardwareMap hardwareMap) {
        MotorEx leftArmMotor = new MotorEx(hardwareMap, "left_arm");
        MotorEx rightArmMotor = new MotorEx(hardwareMap, "right_arm");

        armMotors = new MotorGroup(leftArmMotor, rightArmMotor);

        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setRunMode(Motor.RunMode.PositionControl);
    }

    public void toPositionAtSpeed(int position, int speed) {
        armMotors.setTargetPosition(position);

        while (!armMotors.atTargetPosition()) {
            armMotors.set(armFeedforward.calculate(armMotors.getCurrentPosition(), armMotors.getVelocity()));
        }

        armMotors.stopMotor();
    }
}
