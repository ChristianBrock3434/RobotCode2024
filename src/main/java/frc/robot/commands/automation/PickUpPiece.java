package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickUpPiece extends SequentialCommandGroup{
    public final double actuationPosition = 95 * actuationTicksPerDegree;

    public PickUpPiece() {
        addCommands(
            actuation.setPositionCommand(actuationPosition),
            intake.runVoltageCommand(3),
            actuation.waitUntilAtPosition(actuationPosition),
            intake.waitUntilTripped(),
            new InstantCommand(intake::stopIntakeMotor, intake),
            actuation.setPositionCommand(-60 * actuationTicksPerDegree) //60
        );
    }
}
