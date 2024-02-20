package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilNotMoving extends Command {

    public WaitUntilNotMoving() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
        // return !drivetrain.isMoving();s
    }
}
