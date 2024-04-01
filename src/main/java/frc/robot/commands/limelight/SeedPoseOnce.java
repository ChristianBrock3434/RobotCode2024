package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SeedPoseOnce extends Command {
    double previousSeedTime = -10;
    
    public SeedPoseOnce() {
        addRequirements(limelightShooter);
    }

    @Override
    public void initialize() {
        limelightShooter.turnOnLimelight();
    }

    @Override
    public void execute() {
        // limelightShooter.estimatePose();
        boolean rejectFrontLLUpdate = false;
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
            if (!rejectFrontLLUpdate && !(poseEstimate_FrontLL.pose.getX() == 0) && !(poseEstimate_FrontLL.pose.getY() == 0) && poseEstimate_FrontLL.timestampSeconds - previousSeedTime > 0.25) {
                drivetrain.setPose(poseEstimate_FrontLL.pose, poseEstimate_FrontLL.timestampSeconds);
                previousSeedTime = poseEstimate_FrontLL.timestampSeconds;
                // System.out.println("Seeded at " + previousSeedTime);
                // drivetrain.resetOrientation(poseEstimate_FrontLL.pose);
            }
        } catch (Exception e) {
            // System.out.println(e);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        // limelightShooter.turnOffLimelight();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
