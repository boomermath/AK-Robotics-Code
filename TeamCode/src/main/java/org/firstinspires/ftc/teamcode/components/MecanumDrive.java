package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private final MotorEx motorFrontLeft;
    private final MotorEx motorFrontRight;
    private final MotorEx motorBackLeft;
    private final MotorEx motorBackRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        motorFrontLeft = new MotorEx(hardwareMap, "motor_front_left");
        motorFrontRight = new MotorEx(hardwareMap, "motor_front_right");
        motorBackLeft = new MotorEx(hardwareMap, "motor_back_left");
        motorBackRight = new MotorEx(hardwareMap, "motor_back_right");

        motorFrontLeft.setRunMode(Motor.RunMode.RawPower);
        motorFrontRight.setRunMode(Motor.RunMode.RawPower);
        motorBackLeft.setRunMode(Motor.RunMode.RawPower);
        motorBackRight.setRunMode(Motor.RunMode.RawPower);

        motorFrontLeft.setInverted(true);
        motorBackLeft.setInverted(true);
    }

    private double[] normalizeSpeeds(double ...speeds) {
        double max = Math.abs(speeds[0]);
        double[] normalizedSpeeds = new double[speeds.length];

        for (double s : speeds) {
            max = Math.max(max, Math.abs(s));
        }

        for (int i = 0; i < speeds.length; i++) {
            normalizedSpeeds[i] = speeds[i] / max;
        }

        return normalizedSpeeds;
    }

    public void joystickDrive(double verticalMovement, double horizontalMovement, double yaw) {
        double leftFrontPower  = verticalMovement + horizontalMovement + yaw;
        double rightFrontPower = verticalMovement - horizontalMovement - yaw;
        double leftBackPower   = verticalMovement - horizontalMovement + yaw;
        double rightBackPower  = verticalMovement + horizontalMovement - yaw;

        double[] normalizedSpeeds = normalizeSpeeds(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        leftFrontPower = normalizedSpeeds[0];
        rightFrontPower = normalizedSpeeds[1];
        leftBackPower = normalizedSpeeds[2];
        rightBackPower = normalizedSpeeds[3];

        motorFrontLeft.set(leftFrontPower);
        motorFrontRight.set(rightFrontPower);
        motorBackLeft.set(leftBackPower);
        motorBackRight.set(rightBackPower);
    }
}
