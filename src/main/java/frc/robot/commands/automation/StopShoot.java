package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopShoot extends ParallelCommandGroup{
    
    public StopShoot(double angle) {
        addCommands(
            new InstantCommand(shooter::stopShooter, shooter),
            new InstantCommand(indexer::stopIndexerMotor, indexer),
            new InstantCommand(intake::stopIntakeMotor, intake),
            angleController.setPositionCommand(angle)
        );
    }
}
