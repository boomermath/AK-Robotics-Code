package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {
    private final SimpleServo servo;

    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 1.0;

    public Claw(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "claw_servo", MIN_POSITION, MAX_POSITION, AngleUnit.DEGREES);
    }

   public Command open() {
        return new InstantCommand(() -> servo.turnToAngle(MIN_POSITION));
   }

   public Command close() {
        return new InstantCommand(() -> servo.turnToAngle(MAX_POSITION));
   }
}
