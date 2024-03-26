package frc.robot.commands.drivetrain;

import static frc.robot.Constants.MaxAngularRate;
import static frc.robot.Constants.MaxSpeed;
import static frc.robot.Constants.controller;
import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class represents a command for performing an automatic turn in the drivetrain.
 */
public class AutoTurn extends Command{
    private static boolean isFinished = false;

    private static final double maxAngularRate = Math.PI * 1.3;

    private PIDController rotController = new PIDController(0.25, 0.21, 0.023); //0.25 .21 .023
    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;
    private Rotation2d angle;

    public AutoTurn(double angle) {
        this.angle = Rotation2d.fromDegrees(angle);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // drivetrain.setBrakeMode();
        // limelightIntake.turnOnLimelight();
        // limelightIntake.setLimelightPipeline(LimelightIntake.Pipeline.Note);
        isFinished = false;

        rotController.setSetpoint(angle.getDegrees());
        rotController.setTolerance(0.1);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        output = rotController.calculate(drivetrain.getRotation().getDegrees());
        // System.out.println("Output before modify: " + output);
        if (output > maxAngularRate) {
            output = maxAngularRate;
        } else if (output < -maxAngularRate) {
            output = -maxAngularRate;
        } else if (rotController.atSetpoint()) {
            output = 0;
        }
        // System.out.println("Output after modify: " + output);

        // System.out.println("Current Angle: " + drivetrain.getRotation().getDegrees());
        // System.out.println("Speed: " + output);
        // System.out.println("Setpoint: " + rotController.getSetpoint());

        drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(output) // Drive counterclockwise with negative X (left)
        ).execute();
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
        // drivetrain.setCoastMode();
    }

    @Override
    public boolean isFinished() {
      return isFinished;
        // return false;
    }

    /*
     * sets the isFinished boolean to true, ending the command
     */
    public static void stopCommand() {
        isFinished = true;
    }
}
