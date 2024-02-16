package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.ActuationConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StopIntake extends SequentialCommandGroup {
    public StopIntake() {
        addCommands(
            new InstantCommand(intake::stopIntakeMotor, intake),
            actuation.setPositionCommand(actuationTuckPosition)
        );
    }    
}
