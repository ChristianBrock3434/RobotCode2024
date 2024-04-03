package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;
import frc.robot.commands.limelight.LineUpToNote;

/**
 * This class represents a command group that aligns the robot to pick up a piece.
 * It consists of a series of commands that are executed in parallel or sequentially.
 */
public class LineUpPickUp extends ParallelDeadlineGroup {

    private static final SwerveRequest.FieldCentric driverDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private static final SwerveRequest.RobotCentric autoDrive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Constructs a new instance of the LineUpPickUp command group.
     * It includes commands for picking up a piece and aligning the robot to the target.
     */
    public LineUpPickUp() {
        super(
            new PickUpPiece(intakeVoltage),
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
                    new ShakeController(1.0, 1.0),
                    drivetrain.applyRequest(() -> driverDrive.withVelocityX(-controller.getHID().getRightY() * MaxSpeed)
                        .withVelocityY(-controller.getHID().getRightX() * MaxSpeed)
                        .withRotationalRate(-controller.getHID().getLeftX() * MaxAngularRate)
                    )
                ), 
                () -> !limelightIntake.getTX().equals(Double.NaN)
            )
        );
    }
}
