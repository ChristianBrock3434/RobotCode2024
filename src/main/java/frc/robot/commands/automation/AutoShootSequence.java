package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoShootSequence extends SequentialCommandGroup {
    
    public AutoShootSequence(DoubleSupplier angle, DoubleSupplier velocity, double restingAngle) {
        addCommands(
            angleController.setPositionCommandSupplier(angle),
            shooter.speedUpShooterSupplier(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            actuation.waitUntilAtPosition(actuationTuckPosition),
            intake.startFeedingCommand(feedVelocity, feedAcceleration),
            new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
            new StopShoot(restingAngle)
        );
    }
}
