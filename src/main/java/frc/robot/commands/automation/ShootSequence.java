package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShakeController;

public class ShootSequence extends ConditionalCommand {
    
    public ShootSequence(DoubleSupplier angle, DoubleSupplier velocity) {
        super(
            new SequentialCommandGroup(
                angleController.setPositionCommandSupplier(angle),
                shooter.speedUpShooterSupplier(velocity, shooterSequenceAcceleration),
                angleController.waitUntilAtPositionSupplier(angle),
                shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.8),
                indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
                shooter.checkIfAtSpeedSupplier(velocity),
                indexer.checkIfAtSpeedSupplier(() -> indexerVelocity),
                intake.feedCommand(feedVelocity, feedAcceleration)
            ),
            new ShakeController(1.0),
            // I was held against my will by the compiler
            () -> !(((Double) angle.getAsDouble()).equals(Double.NaN)) || !(((Double) velocity.getAsDouble()).equals(Double.NaN))
        );
    }
}
