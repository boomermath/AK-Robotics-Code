package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Slide extends SubsystemBase {
    private final ServoEx leftServo;
    private final ServoEx rightServo;

    private static final SubsystemBase fillerSub = new SubsystemBase() {
        @Override
        public String getName() {
            return "filler";
        }
    };

    private static final double MIN_LEFT_RETRACT_POSITION = 0;
    private static final double MAX_LEFT_EXTEND_POSITION = 350;

    private static final double MIN_RIGHT_RETRACT_POSITION = 640;
    private static final double MAX_RIGHT_EXTEND_POSITION = 0;
    private static final double ROTATION_INCREMENT = 0.5;

    private static final int LEFT_MULTIPLIER = MIN_LEFT_RETRACT_POSITION - MAX_LEFT_EXTEND_POSITION < 0 ? -1 : 1;
    private static final int RIGHT_MULTIPLIER = MIN_RIGHT_RETRACT_POSITION - MAX_RIGHT_EXTEND_POSITION < 0 ? 1 : -1;

    public Slide(HardwareMap hardwareMap) {
        leftServo = new SimpleServo(hardwareMap, "left_slide_servo", 0,  1000, AngleUnit.DEGREES);
        rightServo = new SimpleServo(hardwareMap, "right_slide_servo", 0, 1000, AngleUnit.DEGREES);
    }

    public ServoEx getLeftServo() {
        return leftServo;
    }

    public ServoEx getRightServo() {
        return rightServo;
    }


    private Command extendLeft() {
        return new InstantCommand(() -> {
            leftServo.turnToAngle(MAX_LEFT_EXTEND_POSITION);
        }, this);
    }

    private Command extendRight() {
        return new InstantCommand(() -> {
            rightServo.turnToAngle(MAX_RIGHT_EXTEND_POSITION);
        }, fillerSub);
    }

    // max extension
    public Command extend() {
        return extendLeft().alongWith(extendRight());
    }

    private Command retractLeft() {
        return new InstantCommand(() -> {
            leftServo.turnToAngle(MIN_LEFT_RETRACT_POSITION);
        }, this);
    }

    private Command retractRight() {
        return new InstantCommand(() -> {
            rightServo.turnToAngle(MIN_RIGHT_RETRACT_POSITION);
        }, fillerSub);
    }

    // max retraction
    public Command retract() {
        return retractLeft().alongWith(retractRight());
    }

    private Command moveForwardLeft() {
        return new InstantCommand(() -> {
            leftServo.rotateByAngle(-LEFT_MULTIPLIER * ROTATION_INCREMENT);
        }, this);
    }

    private Command moveForwardRight() {
        return new InstantCommand(() -> {
            rightServo.turnToAngle(-RIGHT_MULTIPLIER * ROTATION_INCREMENT);
        }, fillerSub);
    }

    // incremental retraction
    public Command moveForward() {
        return moveForwardLeft().alongWith(moveForwardRight());
    }

    private Command moveBackwardLeft() {
        return new InstantCommand(() -> {
            leftServo.rotateByAngle(LEFT_MULTIPLIER * ROTATION_INCREMENT);
        }, this);
    }

    private Command moveBackwardRight() {
        return new InstantCommand(() -> {
            rightServo.turnToAngle(RIGHT_MULTIPLIER * ROTATION_INCREMENT);
        }, fillerSub);
    }


    public Command moveBackward() {
        return moveBackwardLeft().alongWith(moveBackwardRight());
    }
}
