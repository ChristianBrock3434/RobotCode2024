package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class represents an autonomous shoot sequence without stopping.
 * It extends the SequentialCommandGroup class and contains a series of commands
 * that are executed sequentially to perform the shoot sequence.
 *
 * @param angle A DoubleSupplier that provides the angle for the angle controller.
 * @param velocity A DoubleSupplier that provides the velocity for the shooter.
 * @param restingAngle The resting angle for the actuation.
 */
public class AutoShootSequenceNoStop extends SequentialCommandGroup {
    
    public AutoShootSequenceNoStop(DoubleSupplier angle, DoubleSupplier velocity, double restingAngle) {
        addCommands(
            angleController.setPositionCommandSupplier(angle),
            shooter.speedUpShooterSupplier(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.8),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            actuation.waitUntilAtPosition(actuationTuckPosition),
            intake.startFeedingCommand(feedVelocity, feedAcceleration),
            new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
            intake.stopIntakeCommand(),
            actuation.setPositionCommand(restingAngle)
        );
    }
}
