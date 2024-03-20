package frc.robot.commands.automation;

import static frc.robot.Constants.ShooterConstants.shooterSequenceAcceleration;
import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.AutoTurn;

/**
 * A command that prepares the robot for shooting by controlling the rotation, shooter speed, and angle.
 */
public class PrepareForShoot extends ConditionalCommand {
    /**
     * Constructs a new PrepareForShoot command.
     * 
     * @param rotation The rotation value for the AutoTurn command.
     * @param angle The angle value supplied by the DoubleSupplier.
     * @param speed The speed value supplied by the DoubleSupplier.
     */
    public PrepareForShoot(Double rotation, DoubleSupplier angle, DoubleSupplier speed){
        super(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new AutoTurn(-rotation), 
                    new AutoTurn(rotation), 
                    () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
                ),
                shooter.speedUpShooterSupplier(speed, shooterSequenceAcceleration),
                angleController.setPositionCommandSupplier(angle)
            ),
            new ParallelCommandGroup(
                shooter.speedUpShooterSupplier(speed, shooterSequenceAcceleration),
                angleController.setPositionCommandSupplier(angle)
            ),
            () -> !rotation.isNaN()
        );
    }
}
