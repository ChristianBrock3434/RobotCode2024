package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;
import frc.robot.commands.limelight.LineUpToNote;

public class LineUpPickUp extends ParallelDeadlineGroup {

    private static final SwerveRequest.FieldCentric driverDrive = new SwerveRequest.FieldCentric() // I want field-centric
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop
    
    private static final SwerveRequest.RobotCentric autoDrive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public LineUpPickUp() {
        super(
            new PickUpPiece(),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    limelightIntake.prepareForNote(),
                    new LineUpToNote(),
                    actuation.waitUntilAtPosition(actuationPickUpPosition),
                    drivetrain.applyRequest(() -> autoDrive.withVelocityX(-2)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                    )
                ), 
                new ParallelCommandGroup(
                    new ShakeController(0.2),
                    drivetrain.applyRequest(() -> driverDrive.withVelocityX(-controller.getRightY() * MaxSpeed) // Drive forward with
                                                                                                       // negative Y (forward)
                        .withVelocityY(-controller.getRightX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    )
                ), 
                () -> limelightIntake.getTX() != Double.NaN
            )
        );
    }
}
