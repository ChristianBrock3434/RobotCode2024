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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;

public class AutoTurnToGoal extends Command {
    private long startingTime;
    private boolean isWaiting = false;
    private boolean shouldFlip = false;
    private DoubleSupplier multiplier = () -> 1;

    double thetaVelocity;

    protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          22.5,
          0.1,
          2.0,
          new TrapezoidProfile.Constraints(8, Double.MAX_VALUE));

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    protected Supplier<Rotation2d> angleSupplier;

    protected double xSpeaker = 0;
    protected double ySpeaker = 0;

    public AutoTurnToGoal(DoubleSupplier offset) {

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            System.out.println("The Alliance is empty, Please Select an Alliance");
        } else if (alliance.get().equals(Alliance.Red)) {
            xSpeaker = FieldConstants.redSpeakerX;
            ySpeaker = FieldConstants.redSpeakerY;
            multiplier = () -> -1;
            shouldFlip = true;
        } else if (alliance.get().equals(Alliance.Blue)) {
            xSpeaker = FieldConstants.blueSpeakerX;
            ySpeaker = FieldConstants.blueSpeakerY;
            multiplier = () -> 1;
            shouldFlip = false;
        }

        this.angleSupplier = () -> {
          Transform2d translation =
              new Transform2d(
                  xSpeaker - drivetrain.getPose().getX(),
                  ySpeaker - drivetrain.getPose().getY(),
                  new Rotation2d());
          return new Rotation2d(Math.atan2(translation.getY(), translation.getX()) + Units.degreesToRadians(offset.getAsDouble() * multiplier.getAsDouble()));
        };

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // drivetrain.setBrakeMode();
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            System.out.println("The Alliance is empty, Please Select an Alliance");
        } else if (alliance.get().equals(Alliance.Red)) {
            xSpeaker = FieldConstants.redSpeakerX;
            ySpeaker = FieldConstants.redSpeakerY;
            multiplier = () -> -1;
            shouldFlip = true;
        } else if (alliance.get().equals(Alliance.Blue)) {
            xSpeaker = FieldConstants.blueSpeakerX;
            ySpeaker = FieldConstants.blueSpeakerY;
            multiplier = () -> 1;
            shouldFlip = false;
        }

        thetaController.reset(drivetrain.getRotation().getRadians());
        thetaController.enableContinuousInput(-Math.PI, + Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(1.5));
    }
    
    @Override
    public void execute() {
        double targetDirection;
        if (shouldFlip) {
            targetDirection = Rotation2d.fromDegrees(180).minus(angleSupplier.get()).times(-1).getRadians();
        } else {
            targetDirection = angleSupplier.get().getRadians();
        }
        // System.out.println("x: " + translation.getX());
        // System.out.println("y: " + translation.getY());
        // System.out.println("Rot: " + Units.radiansToDegrees(targetDirection));
        // System.out.println("RealRot: " + drivetrain.getDegrees());

        // System.out.println("Offset: " + offset.getAsDouble() * multiplier.getAsDouble());
        // System.out.println("Multi: " + multiplier.getAsDouble());

        
        // double targetDirection = angleSupplier.get().getRadians();
        SmartDashboard.putNumber("x: ", xSpeaker - drivetrain.getPose().getX());
        SmartDashboard.putNumber("y: ", ySpeaker - drivetrain.getPose().getY());
        SmartDashboard.putNumber("Target Rot:", Units.radiansToDegrees(targetDirection));

        thetaVelocity =
            thetaController.calculate(
                drivetrain.getRotation().getRadians(), targetDirection);

        if (thetaController.atSetpoint()) {
            thetaVelocity = 0;
        }

        // System.out.println(thetaVelocity);

        drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getHID().getRightY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(-controller.getHID().getRightX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(thetaVelocity) // Drive counterclockwise with negative X (left)
        ).execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
        // drivetrain.setCoastMode();
    }
    
    @Override
    public boolean isFinished() {
        // If it's at setpoint for 1/8 of a second
        if (thetaController.atSetpoint()) {
            if (!isWaiting) {
                startingTime = System.currentTimeMillis();
                isWaiting = true;
            }
            if (System.currentTimeMillis() - startingTime > 128) {
                return true;
            }
        } else {
            isWaiting = false;
        }
        return false;
    }
}
