package frc.robot.commands.drivetrain;

import static frc.robot.Constants.MaxAngularRate;
import static frc.robot.Constants.MaxSpeed;
import static frc.robot.Constants.controller;
import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command is responsible for controlling the drivetrain to perform position-based turning.
 * It uses a PID controller to calculate the desired heading and applies the necessary velocity and rotational rate to achieve the desired turning motion.
 */
public class DrivePosTurning extends Command{
    private static boolean isFinished = false;
    private static double desiredHeading;

    private static final double maxAngularRate = Math.PI * 2.5;

    private PIDController rotController = new PIDController(0.14, 0.21, 0.003); //0.25 .21 .023
    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);

    private double turningDeadband = 0.25;

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double output;

    public DrivePosTurning() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // limelightIntake.turnOnLimelight();
        // limelightIntake.setLimelightPipeline(LimelightIntake.Pipeline.Note);
        isFinished = false;

        // rotController.setSetpoint(angle.getDegrees());
        rotController.setTolerance(1);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        double rotX = controller.getHID().getLeftX();
        double rotY = controller.getHID().getLeftY();

        double currentHeading = -drivetrain.getRotation().getDegrees();

        if (Math.abs(rotX) > turningDeadband || Math.abs(rotY) > turningDeadband) {
            desiredHeading = Math.toDegrees(Math.atan2(rotY, rotX)) - 90;
            // System.out.println("rotX: " + rotX);
            // System.out.println("rotY: " + rotY);
            // System.out.println("Desired heading: " + desiredHeading);
            while (desiredHeading > 180) {
                desiredHeading -= 360;
            }
            while (desiredHeading < -180) {
                desiredHeading += 360;
            }

            // desiredHeading *= -1;

        } else {
            desiredHeading = currentHeading;
        }

        // rotController.setSetpoint(desiredHeading);
        output = -rotController.calculate(currentHeading, desiredHeading);
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
        // System.out.println("Setpoint: " + desiredHeading);

        drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getHID().getRightY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(-controller.getHID().getRightX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(output) // Drive counterclockwise with negative X (left)
        ).execute();
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
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
