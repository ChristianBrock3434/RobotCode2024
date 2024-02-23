package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ShakeController extends Command{
    private double startingTime;
    private double rumbleIntensity;
    private double shakeTime;

    public ShakeController(double rumbleIntensity, double shakeTime) {
        this.rumbleIntensity = rumbleIntensity;
        this.shakeTime = shakeTime;
    }

    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
        controller.getHID().setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startingTime > shakeTime * 1000){
            return true;
        }
        return false;
    }
}
