package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightShooter;

public class LineUpToGoal extends Command{
    private PIDController lineUPController = new PIDController(0.11, 0.07, 0.002);

    private SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;

    public LineUpToGoal() {
        addRequirements(drivetrain, limelightShooter);
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

        lineUPController.setSetpoint(0);
        lineUPController.setTolerance(2.5);

        lineUPController.calculate(limelightShooter.getTX());
    }

    @Override
    public void execute() {
        output = lineUPController.calculate(limelightShooter.getTX());

        if (lineUPController.atSetpoint() || limelightShooter.getTX() == -1) {
            output = 0;
        }

        // System.out.println("X val: " + limelightShooter.getTX());
        // System.out.println("output: " + output);

        drivetrain.applyRequest(() -> drive.withRotationalRate(output)).execute();
    }

    @Override
    public void end(boolean interrupted) {
        // limelightShooter.turnOffLimelight();
        System.out.println("Made Point");
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        return lineUPController.atSetpoint();
        // return false;
    }
}
