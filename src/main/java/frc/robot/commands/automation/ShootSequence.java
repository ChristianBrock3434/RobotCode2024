package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootSequence extends SequentialCommandGroup{
    
    public ShootSequence() {
        addCommands(
            shooter.speedUpShooter(shooterSequenceVelocity, shooterSequenceAcceleration),
            shooter.checkIfAtSpeed(shooterSequenceVelocity * 0.8),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeed(shooterSequenceVelocity),
            intake.feedCommand(feedVelocity, feedAcceleration)
        );
    }
}
