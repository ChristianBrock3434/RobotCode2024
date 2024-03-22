package frc.robot.commands.automation;

import static frc.robot.Constants.AngleControllerConstants.angleRestingPosition;
import static frc.robot.Subsystems.angleController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class represents a command group that zeros the angle of the shooter.
 * It consists of a sequence of commands that perform the necessary actions to zero the angle.
 */
public class ZeroAngle extends SequentialCommandGroup {
 
    public ZeroAngle() {
        addCommands(
            new InstantCommand(angleController::runDown),
            angleController.waitUntilPressed().withTimeout(4),
            new InstantCommand(angleController::zeroOnSensor),
            angleController.setPositionCommand(angleRestingPosition)  
        );
    }
}
