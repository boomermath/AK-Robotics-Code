package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.components.subsystem.Targetable;

public class Arm implements Targetable {
    private final MotorEx armMotor;
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0.4, 0.1);
    private static final double TICKS_PER_DEGREE = 1.44;

    public Arm(HardwareMap hardwareMap) {
        armMotor = new MotorEx(hardwareMap, "arm_motor");

        armMotor.resetEncoder();

        armMotor.setPositionTolerance(3);

        armMotor.setInverted(true);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setArmFeedforward(ArmFeedforward feedforward) {
        this.armFeedforward = feedforward;
    }

    public Command moveArm(double speed) {
        return new StartEndCommand(() -> {
            armMotor.setRunMode(Motor.RunMode.VelocityControl);
            armMotor.set(speed);
        }, () -> armMotor.set(armFeedforward.calculate(armMotor.getCurrentPosition(), 0, 0)), this);
    }


    @Override
    public Motor getMotor() {
        return armMotor;
    }

    @Override
    public void changePosition(double t) {
        armMotor.set(armFeedforward.calculate(Math.toRadians(t * TICKS_PER_DEGREE), 2, 3));
    }
}
