package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightIntake;

public class LineUpToNote extends Command{
    private PIDController lineUPController = new PIDController(0.04, 0, 0);

    private SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;

    public LineUpToNote() {
        addRequirements(drivetrain, limelightIntake);
    }

    @Override
    public void initialize() {
        limelightIntake.turnOnLimelight();
        limelightIntake.setLimelightPipeline(LimelightIntake.Pipeline.Note);

        lineUPController.setSetpoint(0);
        lineUPController.setTolerance(1.5);
    }

    @Override
    public void execute() {
        limelightIntake.updateLimeLight();

        output = -lineUPController.calculate(limelightIntake.getTX());

        if (lineUPController.atSetpoint()) {
            output = 0;
        }

        // System.out.println("X val: " + limelightIntake.getTX());
        // System.out.println("output: " + output);

        drivetrain.applyRequest(() -> drive.withVelocityY(output)).execute();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).schedule();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
