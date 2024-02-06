package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ShakeController extends Command{
    private double startingTime;
    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startingTime > 1000){
            return true;
        }
        return false;
    }
}
