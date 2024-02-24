package frc.robot.commands.limelight;

import static frc.robot.Constants.MaxAngularRate;
import static frc.robot.Constants.MaxSpeed;
import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightShooter;

public class LineUpToGoal extends Command{

    private PIDController rotController = new PIDController(0.15, 0.0, 0.005);

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;

    public LineUpToGoal() {
        addRequirements(limelightShooter);
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        var pipeline = LimelightShooter.Pipeline.AprilTag3DBlue;
        if (alliance.isEmpty()) {
            System.out.println("The Alliance is empty, Please Select an Alliance");
        } else if (alliance.get().equals(Alliance.Red)) {
            pipeline = LimelightShooter.Pipeline.AprilTag3DRed;
        } 

        limelightShooter.turnOnLimelight();
        limelightShooter.setLimelightPipeline(pipeline);

        double angleFromGoal = limelightShooter.getAngleFromGoal() + 180;
        if (angleFromGoal > 180) {
            angleFromGoal -= 360;
        } else if (angleFromGoal < -180) {
            angleFromGoal += 360;
        }
        rotController.setSetpoint(angleFromGoal);
        rotController.setTolerance(1.5);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        output = rotController.calculate(drivetrain.getPose().getRotation().getDegrees());
        if (rotController.atSetpoint()) {
            output = 0;
        }

        System.out.println("Current Angle: " + drivetrain.getPose().getRotation().getDegrees());
        System.out.println("Speed: " + output);
        System.out.println("Setpoint: " + rotController.getSetpoint());

        drivetrain.applyRequest(() -> drive.withRotationalRate(output)).execute();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
      return rotController.atSetpoint();
        // return false;
    }
}
