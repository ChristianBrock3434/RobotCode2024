package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickUpPiece extends SequentialCommandGroup{

    public PickUpPiece(double voltage) {
        addCommands(
            actuation.setPositionCommand(actuationPickUpPosition),
            actuation.waitUntilAtPosition(actuationPickUpPosition),
            intake.runVoltageCommand(voltage),
            intake.waitUntilTripped(),
            new InstantCommand(intake::stopIntakeMotor, intake),
            actuation.setPositionCommand(actuationTuckPosition)
        );
    }
}
