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

    default void setPosition(int target) {
        getMotor().setRunMode(Motor.RunMode.PositionControl);
        getMotor().setTargetPosition(target);
    };

    void changePosition(double t);

    Motor getMotor();

    default Command targetTicks(int t) {
        return new ToTargetCommand<>(t, this);
    }
}
