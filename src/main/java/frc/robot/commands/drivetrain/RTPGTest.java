package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class RTPGTest extends Command {
    private DoubleSupplier xGoal;
    private DoubleSupplier yGoal;
    private Supplier<Rotation2d> headingGoal;

    private Pose2d targetPose;
    private PathConstraints constraints;

    public RTPGTest(DoubleSupplier xGoal, DoubleSupplier yGoal, Supplier<Rotation2d> headingGoal) {
        this.xGoal = xGoal;
        this.yGoal = yGoal;
        this.headingGoal = headingGoal;
        addRequirements(drivetrain);
    }

    public RTPGTest(double xGoal, double yGoal, Rotation2d headingGoal) {
        this.xGoal = () -> xGoal;
        this.yGoal = () -> yGoal;
        this.headingGoal = () -> headingGoal;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = new Pose2d(xGoal.getAsDouble(), yGoal.getAsDouble(), headingGoal.get());

        // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
            1.5, 1.5,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: getRobotRelativeSpeeds, driveRobotRelative are not defined in the snippet
        if (!interrupted) {
            // new PathfindHolonomic(targetPose, constraints, 0, null, null, null, null, 0, null)
            // Command pathfindingCommand = new PathfindHolonomic(
            //     targetPose,
            //     constraints,
            //     0.0, // Goal end velocity in m/s. Optional
            //     drivetrain::getPose,
            //     drivetrain::getRobotRelativeSpeeds,
            //     drivetrain::driveRobotRelative,
            //     drivetrain.getPathFollowerConfig(), // HolonomicPathFollowerConfig, see the API or "Follow a single path" example for more info
            //     0.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
            //     drivetrain // Reference to drive subsystem to set requirements 
            // );

            // Command.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
