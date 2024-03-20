package frc.robot.commands.automation;

import static frc.robot.Constants.AngleControllerConstants.angleRestingPosition;
import static frc.robot.Subsystems.angleController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ZeroAngle extends SequentialCommandGroup {
 
    public ZeroAngle() {
        addCommands(
            new InstantCommand(angleController::runDown),
            angleController.waitUntilPressed(),
            new InstantCommand(angleController::zeroOnSensor),
            angleController.setPositionCommand(angleRestingPosition)  
        );
    }
}
