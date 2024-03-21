package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import java.util.ConcurrentModificationException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SeedPoseEstimation extends Command {
    
    public SeedPoseEstimation() {
        // addRequirements(limelightShooter);
    }

    @Override
    public void initialize() {
        limelightShooter.turnOnLimelight();
    }

    @Override
    public void execute() {
        // limelightShooter.estimatePose();
        boolean rejectFrontLLUpdate = false;
        // TODO: Check if not alliance based solves x y being same
        LimelightHelpers.PoseEstimate poseEstimate_FrontLL = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");

        if(poseEstimate_FrontLL.rawFiducials.length == 1){
    
            double ambiguity = poseEstimate_FrontLL.rawFiducials[0].ambiguity;
            double dist = poseEstimate_FrontLL.avgTagDist;

            // System.out.println("Dist: " + dist);

            if(ambiguity >= .6 || dist >= 5){
                rejectFrontLLUpdate = true;
            }
        }

        try {
            if (!rejectFrontLLUpdate && !(poseEstimate_FrontLL.pose.getX() == 0) && !(poseEstimate_FrontLL.pose.getY() == 0)) {
                drivetrain.setPose(poseEstimate_FrontLL.pose);
                // drivetrain.resetOrientation(poseEstimate_FrontLL.pose);
            }
        } catch (Exception e) {
            System.out.println(e);
        }

        // System.out.println("X:" + poseEstimate_FrontLL.pose.getX());
        // System.out.println("Y: " + poseEstimate_FrontLL.pose.getY());
    }

    @Override
    public void end(boolean isInterrupted) {
        // limelightShooter.turnOffLimelight();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
