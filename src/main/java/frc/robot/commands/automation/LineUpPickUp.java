package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.LineUpToNote;

public class LineUpPickUp extends ParallelDeadlineGroup {
    private static SwerveRequest.RobotCentric drive = 
                    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public LineUpPickUp() {
        super(
            new PickUpPiece(),
            new SequentialCommandGroup(
                new LineUpToNote(),
                drivetrain.applyRequest(() -> drive.withVelocityX(0.1).withVelocityY(0).withRotationalRate(0))
            )
        );
    }
}
