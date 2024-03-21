package frc.robot.commands.limelight;

import static frc.robot.Subsystems.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.PIDConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command is used to go to the last place in a path while lining up to a note.
 * It utilizes PID controllers for driving to the end state and correcting the alignment.
 */
public class LineUpWithNotePath extends Command {
    private PIDController m_xPIDController; //1.0
    private PIDController m_yPIDController; //1.0
    private PIDController m_lineUpPIDController; //0.04

    private final String autoPath;
    private final int index;

    private double xGoal;
    private double yGoal;
    private double currentRot;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double xSpeed;
    private double ySpeed;
    private double lineUpCorrection;

    ChassisSpeeds speed;


  /**
   * Goes to last place in a path while lining up to a note
   *
   * @param autoPath The name of your auto to grab the path from in path planner
   * @param index The index of the path in the auto to grab
   * @param drivePIDConstants the PID of driving to your end state
   * @param correctPIDConstants the PID of correction to the piece
   */
  public LineUpWithNotePath(String autoPath, int index, PIDConstants drivePidConstants, PIDConstants correctPidConstants) {
    this.autoPath = autoPath;
    this.index = index;

    this.m_xPIDController = new PIDController(drivePidConstants.kP, drivePidConstants.kI, drivePidConstants.kD);
    this.m_yPIDController = new PIDController(drivePidConstants.kP, drivePidConstants.kI, drivePidConstants.kD);
    this.m_lineUpPIDController = new PIDController(correctPidConstants.kP, correctPidConstants.kI, correctPidConstants.kD);
    addRequirements(drivetrain, limelightIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var state = drivetrain.getEndPath(autoPath, index);
    xGoal = state.positionMeters.getX();
    yGoal = state.positionMeters.getY();

    // System.out.println("xGoal: " + xGoal);
    // System.out.println("yGoal: " + yGoal);

    m_xPIDController.setTolerance(0.1);
    m_xPIDController.setSetpoint(xGoal);

    m_yPIDController.setTolerance(0.1);
    m_yPIDController.setSetpoint(yGoal);

    m_lineUpPIDController.setTolerance(1.5);
    m_lineUpPIDController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    currentRot = drivetrain.getRotation().getRadians();

    xSpeed = m_xPIDController.calculate(drivetrain.getPose().getX());

    if (m_xPIDController.atSetpoint()) {
      xSpeed = 0;
    }

    ySpeed = m_yPIDController.calculate(drivetrain.getPose().getY());

    if (m_yPIDController.atSetpoint()) {
      ySpeed = 0;
    }

    lineUpCorrection = -m_lineUpPIDController.calculate(limelightIntake.getTX());

    if (m_lineUpPIDController.atSetpoint() || limelightIntake.getTX().equals(Double.NaN)) {
      lineUpCorrection = 0;
    }

    // System.out.println("Current rot: " + currentRot);

    // System.out.println("X Speed: " + xSpeed);
    // System.out.println("Y Speed: " + ySpeed);

    // double temp = xSpeed * Math.cos(currentRot) + ySpeed * Math.sin(currentRot);
    // ySpeed = -xSpeed * Math.sin(currentRot) + ySpeed * Math.cos(currentRot);
    // xSpeed = temp;

    // currentRot *= -1;

    speed = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, 0), Rotation2d.fromRadians(currentRot));
    speed = speed.plus(new ChassisSpeeds(0, lineUpCorrection, 0));

    // System.out.println("corrected X Speed: " + xSpeed);
    // System.out.println("corrected Y Speed: " + ySpeed);

    // double correctedYSpeed = ySpeed + lineUpCorrection;

    drivetrain.applyRequest(() -> drive.withVelocityX(speed.vxMetersPerSecond).withVelocityY(speed.vyMetersPerSecond).withRotationalRate(0.0)).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // limelightIntake.turnOffLimelight();
    drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(m_limelight.getTX());
    return m_xPIDController.atSetpoint();
  }
}
