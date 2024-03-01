package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoShootSequenceNoStop extends SequentialCommandGroup {
    
    public AutoShootSequenceNoStop(DoubleSupplier angle, DoubleSupplier velocity, double restingAngle) {
        addCommands(
            // new PrintCommand("Angle: " + angle.getAsDouble()),
            // new PrintCommand("Speed: " + velocity.getAsDouble())
            angleController.setPositionCommandSupplier(angle),
            shooter.speedUpShooterSupplier(velocity, shooterSequenceAcceleration),
            angleController.waitUntilAtPositionSupplier(angle),
            shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.8),
            indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
            shooter.checkIfAtSpeedSupplier(velocity),
            indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
            actuation.waitUntilAtPosition(actuationTuckPosition),
            intake.startFeedingCommand(feedVelocity, feedAcceleration),
            new WaitCommand(1.0).raceWith(shooter.waitUntilRingLeft()),
            actuation.setPositionCommand(restingAngle),
            new InstantCommand(intake::stopIntakeMotor)
            // new StopShoot(restingAngle)
        );
    }
}
