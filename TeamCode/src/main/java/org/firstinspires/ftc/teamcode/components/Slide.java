package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    public CRServo servo;

    public Slide(HardwareMap hardwareMap) {
        servo = new CRServo(hardwareMap, "slide_servo");

        //servo.setFeedforwardCoefficients(0,0,0);
    }

    public void setSlidePosition(int position) {
        servo.setRunMode(Motor.RunMode.PositionControl);
        servo.setTargetPosition(position);

        while (!servo.atTargetPosition()) {
            servo.set(0.5);
        }
    }

    public void moveSlide() {
        servo.setRunMode(Motor.RunMode.VelocityControl);
        servo.set(3);
    }
}
