package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;

public class LineUpToGoal extends Command{
    private PIDController lineUPController = new PIDController(0.1, 0, 0);

    private SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;
    private double offset;

    public LineUpToGoal(double offset) {
        this.offset = offset;
        addRequirements(drivetrain, limelightShooter);
    }

    @Override
    public void initialize() {
        limelightShooter.turnOnLimelight();
        limelightShooter.setLimelightPipeline(LimelightShooter.Pipeline.AprilTag3D);

        lineUPController.setSetpoint(offset);
        lineUPController.setTolerance(1.5);
    }

    @Override
    public void execute() {
        limelightShooter.updateLimeLight();

        output = lineUPController.calculate(limelightShooter.getTX());

        if (lineUPController.atSetpoint() || limelightShooter.getTX() == 0) {
            output = 0;
        }

        // System.out.println("X val: " + limelightShooter.getTX());
        // System.out.println("output: " + output);

        drivetrain.applyRequest(() -> drive.withRotationalRate(output)).execute();
    }

    @Override
    public void end(boolean interrupted) {
        // limelightShooter.turnOffLimelight();
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        //   return lineUPController.atSetpoint();
        return false;
    }
}
