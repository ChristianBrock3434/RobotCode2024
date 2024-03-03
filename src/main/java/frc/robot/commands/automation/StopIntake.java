package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.ActuationConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopIntake extends ParallelCommandGroup {
    public StopIntake() {
        addCommands(
            intake.stopIntakeCommand(),
            actuation.setPositionCommand(actuationTuckPosition)
        );
    }    
}
