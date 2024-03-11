package frc.robot.commands.limelight;

import static frc.robot.Constants.controller;
import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.subsystems.LimelightShooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

// Get tag number, choose from enum for tag, move to x and y via limelight, rotation via pigey
//15 x: 8.55
//15 y: 14.11
public class LineUpToTrap extends Command {
    private static boolean isFinished = false;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(1);

    // private ProfiledPIDController yController = new ProfiledPIDController(0.1, 
    //                                                                     0.01, 
    //                                                                     0.005, 
    //                                                                     new Constraints(5,5));
    // private ProfiledPIDController xController = new ProfiledPIDController(0.1, 
    //                                                                     0.01, 
    //                                                                     0.005, 
    //                                                                     new Constraints(5, 5));
    // private ProfiledPIDController rotController = new ProfiledPIDController(0.14, 
    //                                                                     0.21, 
    //                                                                     0.003, 
    //                                                                     new Constraints(5, 5));
    private PIDController yController = new PIDController(0.08, 0.01, 0.005);
    private PIDController xController = new PIDController(0.08, 0.01, 0.005);
    private PIDController rotController = new PIDController(0.14, 0.21, 0.003);

    private SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    private static enum Tag {
        Blue1Trap(15, 16.3, 12.8, -59.7), //12, 10.7, -61.7
        Blue2Trap(16, 16.3, 12.8, -59.7),
        Blue3Trap(14, 16.3, 12.8, -59.7),
        Red1Trap(12, 16.3, 12.8, -59.7),
        Red2Trap(11, 16.3, 12.8, -59.7),
        Red3Trap(13, 16.3, 12.8, -59.7);

        public int tagNum;
        public Double x;
        public Double y;
        public Double rot;

        private Tag(int tagNum, Double x, Double y, Double rot){
            this.tagNum = tagNum;
            this.x = x;
            this.y = y;
            this.rot = rot;
        }
    }

    public LineUpToTrap() {
        addRequirements(drivetrain, limelightShooter);
    }

    @Override
    public void initialize() {
        isFinished = false;

        System.out.println("Start Line Up");
        limelightShooter.turnOnLimelight();
        limelightShooter.setLimelightPipeline(LimelightShooter.Pipeline.Trap);

        yController.setTolerance(0.25);

        xController.setTolerance(0.5);

        rotController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        // Get tag number
        Double id = limelightShooter.getApriltagID();
        if (id.isNaN() || limelightShooter.getTX().isNaN() || limelightShooter.getTY().isNaN()) {
            drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
            controller.getHID().setRumble(RumbleType.kRightRumble, 0.5);
        } else {
            controller.getHID().setRumble(RumbleType.kRightRumble, 0);
        }
        // Choose from enum for tag
        Tag tag = null;
        for (Tag t : Tag.values()) {
            if (t.tagNum == id.intValue()) {
                tag = t;
                break;
            }
        }

        if (!(tag == null)) {
            // if (tag.equals(null) || tag.x.isNaN() || tag.y.isNaN() || tag.rot.isNaN()) {
            //     controller.getHID().setRumble(RumbleType.kLeftRumble, 0.5);
            //     return;
            // } else {
            //     controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
            // }

            // Move to x and y via limelight
            xSpeed = -xLimiter.calculate(controller.getRightX());
            ySpeed = -yLimiter.calculate(controller.getRightY());

            xController.setSetpoint(tag.x);
            xController.calculate(limelightShooter.getTX());

            yController.setSetpoint(tag.y);
            yController.calculate(limelightShooter.getTY());

            if (xController.atSetpoint() && yController.atSetpoint()) {
                controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
                xSpeed = 0;
                ySpeed = 0;
            }

            // Rotation via pigey
            rotController.setSetpoint(tag.rot);
            rotSpeed = rotController.calculate(drivetrain.getRotation().getDegrees());
            if (rotController.atSetpoint()) {
                rotSpeed = 0;
            }

            // System.out.println("TX: " + limelightShooter.getTX());
            // System.out.println("TY: " + limelightShooter.getTY());

            // apply to swerve
            drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed).withVelocityY(xSpeed).withRotationalRate(rotSpeed)).execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kRightRumble, 0);
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
        System.out.println("End Line up");
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public static void stopCommand() {
        isFinished = true;
    }
}