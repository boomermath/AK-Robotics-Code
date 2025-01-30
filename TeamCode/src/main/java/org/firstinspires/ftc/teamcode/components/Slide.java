package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.components.subsystem.Targetable;

public class Slide extends SubsystemBase implements Targetable {
    public CRServo servo;
    private static final double DEFAULT_SERVO_SPEED = 0.5;

    public Slide(HardwareMap hardwareMap) {
        servo = new CRServo(hardwareMap, "slide_servo");
        servo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    private void setSlideSpeed(boolean reversed) {
        servo.setRunMode(Motor.RunMode.VelocityControl);
        servo.set(DEFAULT_SERVO_SPEED * (reversed ? 1 : -1));
    }

    public Command moveSlide(boolean reverse) {
        return new StartEndCommand(() -> setSlideSpeed(reverse), servo::stop, this);
    }

    @Override
    public void changePosition(double t) {
        servo.set(DEFAULT_SERVO_SPEED);
    }

    @Override
    public Motor getMotor() {
        return servo;
    }
}
