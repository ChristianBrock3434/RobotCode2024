package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;

/**
 * A command that represents the sequence of actions required to perform a shooting operation.
 * This command includes setting the angle, speeding up the shooter, waiting until the angle is reached,
 * checking if the shooter is at the desired speed, speeding up the indexer, checking if the indexer is at the desired speed,
 * and feeding the intake.
 */
public class ShootSequence extends ConditionalCommand {
    
    /**
     * Constructs a new ShootSequence command.
     * 
     * @param angle    a supplier for the desired angle of the shooter
     * @param velocity a supplier for the desired velocity of the shooter
     */
    public ShootSequence(DoubleSupplier angle, DoubleSupplier velocity) {
        super(
            new SequentialCommandGroup(
                angleController.setPositionCommandSupplier(angle),
                shooter.speedUpShooter(velocity, shooterSequenceAcceleration),
                angleController.waitUntilAtPositionSupplier(angle),
                shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.8),
                indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
                shooter.checkIfAtSpeedSupplier(velocity),
                indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
                intake.feedCommand(feedVelocity, feedAcceleration)
            ),
            new ShakeController(1.0, 1.0),
            // I was held against my will by the compiler
            () -> !(((Double) angle.getAsDouble()).equals(Double.NaN)) || !(((Double) velocity.getAsDouble()).equals(Double.NaN))
        );
    }
}
