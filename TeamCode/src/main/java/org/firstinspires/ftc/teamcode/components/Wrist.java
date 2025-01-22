package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Wrist extends SubsystemBase {
    private final ServoEx servo;
    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 1;

    public Wrist(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "wrist_servo", MIN_POSITION, MAX_POSITION, AngleUnit.DEGREES);
    }

    public Command rotateBy(double servoAngle) {
        return new InstantCommand(() -> servo.rotateByAngle(servoAngle));
    }

    public Command rotateTo(double servoAngle) {
        return new InstantCommand(() -> servo.turnToAngle(servoAngle));
    }
}
