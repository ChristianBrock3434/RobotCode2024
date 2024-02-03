package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootSequence extends SequentialCommandGroup{
    public static final double shooterVelocity = 60; //45 //60 //10 for amp
    public static final double indexerVelocity = 60; //40 amp //60 shooting
    
    public ShootSequence() {
        addCommands(
            shooter.speedUpShooter(shooterVelocity, 100),
            shooter.checkIfAtSpeed(shooterVelocity * 0.8),
            indexer.speedUpIndexer(indexerVelocity, 100),
            shooter.checkIfAtSpeed(shooterVelocity),
            intake.feedCommand(60, 100)
        );
    }
}
