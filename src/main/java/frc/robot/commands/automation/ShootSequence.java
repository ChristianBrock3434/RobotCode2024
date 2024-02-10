package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.AngleControllerConstants.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;

public class ShootSequence extends ConditionalCommand {
    
    public ShootSequence(DoubleSupplier angle, DoubleSupplier velocity) {
        super(
            new SequentialCommandGroup(
                // new PrintCommand("Angle: " + angle.getAsDouble()),
                // new PrintCommand("Speed: " + velocity.getAsDouble())
                angleController.setPositionCommandSupplier(angle),
                shooter.speedUpShooterSupplier(velocity, shooterSequenceAcceleration),
                angleController.waitUntilAtPositionSupplier(angle, 1),
                shooter.checkIfAtSpeedSupplier(velocity, 0.8),
                indexer.speedUpIndexer(indexerVelocity, indexerAcceleration),
                shooter.checkIfAtSpeedSupplier(velocity, 1.0),
                intake.feedCommand(feedVelocity, feedAcceleration)
            ),
            new ShakeController(),
            () -> (angle.getAsDouble() >= 0) && (velocity.getAsDouble() >= 0)
        );
    }
}
