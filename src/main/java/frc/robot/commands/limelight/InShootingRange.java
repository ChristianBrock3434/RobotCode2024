package frc.robot.commands.limelight;

import static frc.robot.Subsystems.limelightShooter;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightShooter;

/**
 * A command that checks if the robot is in the shooting range based on the distance from the goal.
 */
public class InShootingRange extends Command {
    private static boolean isFinished = false;

    private static final double deadzone = 0.15;

    public InShootingRange() {
        addRequirements(limelightShooter);
    }

    @Override
    public void initialize() {
        isFinished = false;

        // var alliance = DriverStation.getAlliance();
        // var pipeline = LimelightShooter.Pipeline.AprilTag3DBlue;
        // if (alliance.isEmpty()) {
        //     System.out.println("The Alliance is empty, Please Select an Alliance");
        // } else if (alliance.get().equals(Alliance.Red)) {
        //     pipeline = LimelightShooter.Pipeline.AprilTag3DRed;
        // } else if (alliance.get().equals(Alliance.Blue)) {
        //     pipeline = LimelightShooter.Pipeline.AprilTag3DBlue;
        // }

        limelightShooter.turnOnLimelight();
        // limelightShooter.setLimelightPipeline(pipeline);
    }

    @Override
    public void execute() {
        double distanceFromGoal = limelightShooter.getDistanceFromGoal();
        if (distanceFromGoal >= farShotDistance - deadzone && distanceFromGoal <= farShotDistance + deadzone) {
            controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
     
    @Override
    public boolean isFinished() {
        // return true;
        return isFinished;
    }

    /*
     * sets the isFinished boolean to true, ending the command
     */
    public static void stopCommand() {
        isFinished = true;
    }
}
