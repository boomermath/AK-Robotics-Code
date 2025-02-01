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
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.5, 0.4, 0.3);
    private static final double TICKS_PER_DEGREE = 130 / 90.0;

    public Arm(HardwareMap hardwareMap) {
        armMotor = new MotorEx(hardwareMap, "arm_motor");

        armMotor.stopAndResetEncoder();
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
    public void stop() {
        if (getMotor().getCurrentPosition() > 10) {
            armMotor.setRunMode(Motor.RunMode.RawPower);
            armMotor.set(0.2);
        } else {
            armMotor.stopMotor();
        }
    }

    public void holdPosition(double target) {
        armMotor.set(armFeedforward.calculate(Math.toRadians(target * TICKS_PER_DEGREE), 0, 0));
    }

    @Override
    public Motor getMotor() {
        return armMotor;
    }

    @Override
    public void changePosition(double t) {
        double multiplier = t > armMotor.getCurrentPosition() ? 1 : -1;
        armMotor.set(multiplier * armFeedforward.calculate(Math.toRadians(t * TICKS_PER_DEGREE), 4, 3));
    }
}
