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
    private PIDController lineUPController = new PIDController(0.123, 0.0, 0.0125); //Don't use I because of how we deal with Double.NaN
    private SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private double output;

    public LineUpToGoal() {
        addRequirements(limelightShooter); //Not including drivetrain because of charge then shoot when stopped
    }

    @Override
    public void initialize() {
        System.out.println("Made Point");
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
        lineUPController.setTolerance(2.75);

        lineUPController.calculate(1000);
    }

    @Override
    public void execute() {
        if (limelightShooter.getTX().equals(Double.NaN)) {
            lineUPController.calculate(1000);
            output = 0;
        } else {
            output = lineUPController.calculate(limelightShooter.getTX());

            if (lineUPController.atSetpoint()) {
                output = 0;
            }
        }

        System.out.println("X val: " + limelightShooter.getTX());
        System.out.println("output: " + output);

        drivetrain.applyRequest(() -> drive.withRotationalRate(output)).execute();
    }

    @Override
    public void end(boolean interrupted) {
        // limelightShooter.turnOffLimelight();
        System.out.println("Made Point");
        drivetrain.applyRequest(() -> brake).execute();
    }

    @Override
    public boolean isFinished() {
        return lineUPController.atSetpoint();
        // return false;
    }
}
