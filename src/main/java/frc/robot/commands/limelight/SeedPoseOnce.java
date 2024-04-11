package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        Rotation2d rot = drivetrain.getRotation();

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
          DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
        } else if (alliance.get().equals(Alliance.Red)) {
          rot = drivetrain.getRotation().plus(Rotation2d.fromDegrees(-180));
        }

        boolean rejectFrontLLUpdate = false;

        LimelightHelpers.SetRobotOrientation(limelightShooter.LIMELIGHT, rot.getDegrees(), drivetrain.getAngularRate(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate poseEstimate_FrontLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightShooter.LIMELIGHT);

        if(Math.abs(drivetrain.getAngularRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          rejectFrontLLUpdate = true;
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
