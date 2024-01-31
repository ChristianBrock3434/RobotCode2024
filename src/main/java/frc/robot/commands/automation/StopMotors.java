package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StopMotors extends SequentialCommandGroup{
    
    public StopMotors() {
        addCommands(
            new InstantCommand(shooter::stopShooter, shooter),
            new InstantCommand(indexer::stopIndexerMotor, indexer),
            new InstantCommand(intake::stopIntakeMotor, intake)
        );
    }
}
