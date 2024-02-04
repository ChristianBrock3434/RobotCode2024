package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class LineUpWithNotePath extends Command {
    //TODO: Rewrite for x and y movement while y lineup
    private PIDController m_xPIDController = new PIDController(1.0, 0.0, 0.0);
    private PIDController m_yPIDController = new PIDController(0.04, 0.0, 0.0);
    // private PIDController m_rotPIDController = new PIDController(0.01, 0.0, 0.0);

    private final String autoPath;
    private final int index;

    private double xGoal;
    // private double rotGoal;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double xSpeed;
    private double ySpeed;
    private double rotSpeed;


  /**
   * Goes to last place in a path while lining up to a note
   *
   * @param autoPath The name of your auto to grab the path from in path planner
   * @param index The index of the path in the auto to grab
   */
  public LineUpWithNotePath(String autoPath, int index) {
    this.autoPath = autoPath;
    this.index = index;
    addRequirements(drivetrain, limelightIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var state = drivetrain.getEndPath(autoPath, index);
    xGoal = state.positionMeters.getX();
    // rotGoal = state.targetHolonomicRotation.getDegrees();

    m_xPIDController.setTolerance(0.2);
    m_xPIDController.setSetpoint(xGoal);

    m_yPIDController.setTolerance(1.5);
    m_yPIDController.setSetpoint(0);

    // m_rotPIDController.setTolerance(1);
    // m_rotPIDController.setSetpoint(rotGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightIntake.updateLimeLight();

    xSpeed = m_xPIDController.calculate(drivetrain.getPose().getX());

    if (m_xPIDController.atSetpoint()) {
      xSpeed = 0;
    }

    // System.out.println("xGoal: " + xGoal);
    // System.out.println("rotGoal: " + rotGoal);
    // System.out.println(drivetrain.getPose().getX());

    ySpeed = m_yPIDController.calculate(limelightIntake.getTX());

    if (m_yPIDController.atSetpoint()) {
      ySpeed = 0;
    }

    System.out.println("x speed: " + xSpeed);
    System.out.println("y speed: " + ySpeed);

    // rotSpeed = m_rotPIDController.calculate(drivetrain.getRotation().getDegrees());

    // if (m_rotPIDController.atSetpoint()) {
    //   rotSpeed = 0;
    // }

    drivetrain.applyRequest(() -> drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(0.0)).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // limelightIntake.turnOffLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_limelight.getTX());
    if (m_xPIDController.atSetpoint() && m_yPIDController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
