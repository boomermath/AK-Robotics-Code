package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.teamcode.components.subsystem.Targetable;

public class ToTargetCommand<T extends Subsystem & Targetable> extends CommandBase {
    private final T targetSubsystem;
    private final int target;

    public ToTargetCommand(int target, T targetSubsystem) {
        this.targetSubsystem = targetSubsystem;
        this.target = target;

        addRequirements(targetSubsystem);
    }

    @Override
    public void initialize() {
        targetSubsystem.setPosition(target);
    }

    public void execute() {
        targetSubsystem.changePosition(target);
    }

    @Override
    public void end(boolean interrupted) {
       targetSubsystem.stop();
    }

    public boolean isFinished() {
        return targetSubsystem.atPosition();
    }
}
