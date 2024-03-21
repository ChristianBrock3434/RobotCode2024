package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;

public class AutoTurnToGoal extends Command {
    private static boolean isFinished = false;

    protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          6.0,
          0.0,
          0.1,
          new TrapezoidProfile.Constraints(8, 100));

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    protected DoubleSupplier xSupplier;
    protected DoubleSupplier ySupplier;
    protected Supplier<Rotation2d> angleSupplier;

    protected double xSpeaker = 0;
    protected double ySpeaker = 0;

    public AutoTurnToGoal(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            System.out.println("The Alliance is empty, Please Select an Alliance");
        } else if (alliance.get().equals(Alliance.Red)) {
            xSpeaker = FieldConstants.redSpeakerX;
            ySpeaker = FieldConstants.redSpeakerY;
        } else if (alliance.get().equals(Alliance.Blue)) {
            xSpeaker = FieldConstants.blueSpeakerX;
            ySpeaker = FieldConstants.blueSpeakerY;
        }

        this.angleSupplier = () -> {
          Transform2d translation =
              new Transform2d(
                  xSpeaker - drivetrain.getPose().getX(),
                  ySpeaker - drivetrain.getPose().getY(),
                  new Rotation2d());
          return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
        };

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isFinished = false;

        thetaController.reset(drivetrain.getRotation().getRadians());
        thetaController.enableContinuousInput(-Math.PI, + Math.PI);
    }
    
    @Override
    public void execute() {
        Transform2d translation =
              new Transform2d(
                  xSpeaker - drivetrain.getPose().getX(),
                  ySpeaker - drivetrain.getPose().getY(),
                  new Rotation2d());
        double targetDirection = new Rotation2d(Math.atan2(translation.getY(), translation.getX())).getRadians();

        System.out.println("x: " + translation.getX());
        System.out.println("y: " + translation.getY());
        System.out.println("Rot: " + Units.radiansToDegrees(targetDirection));
        System.out.println("RealRot: " + drivetrain.getDegrees());

        
        // double targetDirection = angleSupplier.get().getRadians();

        double thetaVelocity =
            thetaController.calculate(
                drivetrain.getRotation().getRadians(), targetDirection);

        System.out.println(thetaVelocity);

        drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(thetaVelocity) // Drive counterclockwise with negative X (left)
        ).execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        // drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
    }
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    /*
     * sets the isFinished boolean to true, ending the command
     */
    public static void stopCommand() {
        isFinished = true;
    }
}
