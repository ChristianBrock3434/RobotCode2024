package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Subsystems.limelightIntake;
import static frc.robot.Subsystems.limelightShooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;

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
        limelightIntake.setLights(LimelightIntake.LightMode.BLINK);
        limelightShooter.setLights(LimelightShooter.LightMode.BLINK);
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        limelightIntake.setLights(LimelightIntake.LightMode.DEFAULT);
        limelightShooter.setLights(LimelightShooter.LightMode.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startingTime > shakeTime * 1000){
            return true;
        }
        return false;
    }
}
