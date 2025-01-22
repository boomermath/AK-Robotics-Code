package org.firstinspires.ftc.teamcode.components.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.commands.ToTargetCommand;

public interface Targetable extends Subsystem {
    default void stop() {
        getMotor().stopMotor();
    };

    default boolean atPosition() {
        return getMotor().atTargetPosition();
    };

    default void setPosition(double target) {
        getMotor().setRunMode(Motor.RunMode.PositionControl);
        getMotor().set(target);
    };

    void changePosition();

    Motor getMotor();

    default Command target(double t) {
        return new ToTargetCommand<>(t, this);
    }
}
