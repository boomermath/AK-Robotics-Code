package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    public MotorEx liftMotor;
    private static final double DEFAULT_LIFT_SPEED = 0.5;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = new MotorEx(hardwareMap, "lift_motor");

        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public Command lift() {
        return new InstantCommand(() -> liftMotor.set(DEFAULT_LIFT_SPEED));
    }

    public Command stop() {
        return new InstantCommand(() -> liftMotor.set(0));
    }
}