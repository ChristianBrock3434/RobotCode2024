// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.ClimberConstants.maxClimberHeight;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.podiumShotSpeed;
import static frc.robot.Constants.ShooterConstants.subwooferShotSpeed;
// import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.ShootSequence;
import frc.robot.commands.automation.StopIntake;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.ShakeController;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.StopShoot;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.limelight.LineUpToGoal;
import frc.robot.commands.limelight.LineUpWithNotePath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private static SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private static SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

  private static boolean isSubwooferShot = true;
  private static enum ampState {
    IDLE,
    PREPARED,
    SHOOTING;
  };
  private static ampState currentState = ampState.IDLE;

  // TODO: Add on shooting
  // Lock wheels
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // private final SwerveRequest.PointWheelsAt pointForward = new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(0));

  // public boolean intakePosition = false;
  // public boolean tuckPosition = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    linkAutoCommands();
    setUpNetworkTables();
    configureBindings();
  }

  public void setUpNetworkTables() {
    SmartDashboard.putBoolean("Manual Shot", isSubwooferShot);

    SmartDashboard.putBoolean("Left Intake Note Sensor", intake.getLeftNoteSensor());
    SmartDashboard.putBoolean("Right Intake Note Sensor", intake.getRightNoteSensor());

    SmartDashboard.putBoolean("Shooter line break", shooter.getNoteSensor());
  }

  /**
   * link commands to pathplanner for autos
   */
  public void linkAutoCommands() {
    NamedCommands.registerCommand("shoot", new AutoShootSequence(this::getAngle, this::getSpeed, angleRestingPosition));
    NamedCommands.registerCommand("intake", new PickUpPiece(autoIntakeVoltage));

    NamedCommands.registerCommand("shoot1FarBlue", new AutoShootSequence(() -> 20, () -> 65, 31.5));
    NamedCommands.registerCommand("shoot2FarBlue", new AutoShootSequence(() -> 32, () -> 65, 31.5));

    NamedCommands.registerCommand("tuckActuator", actuation.setPositionCommand(actuationTuckPosition));

    NamedCommands.registerCommand("lineUpToNote1CloseBlue", new LineUpWithNotePath("4 ring close blue", 0, new PIDConstants(2.0), new PIDConstants(0.8)));
    NamedCommands.registerCommand("lineUpToNote2CloseBlue", new LineUpWithNotePath("4 ring close blue", 1, new PIDConstants(2.0), new PIDConstants(0.8)));
    NamedCommands.registerCommand("lineUpToNote3CloseBlue", new LineUpWithNotePath("4 ring close blue", 3, new PIDConstants(2.0), new PIDConstants(0.8)));

    NamedCommands.registerCommand("lineUpToNote1FarBlue", new LineUpWithNotePath("3 ring far blue", 0, new PIDConstants(3.0), new PIDConstants(0.8)));
    NamedCommands.registerCommand("lineUpToNote2FarBlue", new LineUpWithNotePath("3 ring far blue", 2, new PIDConstants(2.0), new PIDConstants(0.8)));
    NamedCommands.registerCommand("lineUpToNote3FarBlue", new LineUpWithNotePath("3 ring far blue", 4, new PIDConstants(3.0), new PIDConstants(0.8)));
    
    NamedCommands.registerCommand("prepareForNote", limelightIntake.prepareForNote());

    NamedCommands.registerCommand("waitForPathInterrupt", AutoTracker.waitForSignal(AutoTracker.tracked.PATH));
    NamedCommands.registerCommand("interruptPath", AutoTracker.sendSignal(AutoTracker.tracked.PATH));

    NamedCommands.registerCommand("waitForIntakeSignal", AutoTracker.waitForSignal(AutoTracker.tracked.INTAKE));
    NamedCommands.registerCommand("sendIntakeSignal", AutoTracker.sendSignal(AutoTracker.tracked.INTAKE));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotLimiter.calculate(-controller.getLeftX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // reset the field-centric heading on start button    
    controller.start().onTrue(
      new ParallelCommandGroup(
        drivetrain.runOnce(() -> drivetrain.resetOrientation()),
        new ShakeController(0.5, 0.25)
      )
    );

    controller.rightBumper().whileTrue(new PickUpPiece(intakeVoltage)).onFalse(new StopIntake());
    controller.b().whileTrue(intake.feedCommand(outtakeVelocity, outtakeAcceleration));
    controller.x().whileTrue(intake.feedCommand(intakeVelocity, intakeAcceleration));

    // Revved shot
    controller.rightTrigger(0.1).whileTrue(new ParallelCommandGroup(
      drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed)
            .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed)
            .withRotationalRate(rotLimiter.calculate(-controller.getLeftX()) * MaxAngularRate * 0.5)
      ),
      new SequentialCommandGroup(
      shooter.speedUpShooter(65, 100),
      drivetrain.waitUntilNotMoving(),
      new SequentialCommandGroup(
        // new LineUpToGoal(),
        new ShootSequence(this::getAngle, this::getSpeed) 
      )
    )
    )).onFalse(new StopShoot(angleRestingPosition));

    controller.back().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::changeManualShootMode),
        new ShakeController(0.5, 0.25)
      )
    );

    controller.leftTrigger(0.1)
      .and(() -> isSubwooferShot)
        .whileTrue(
          new ParallelCommandGroup(
            drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed)
                .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed)
                .withRotationalRate(rotLimiter.calculate(-controller.getLeftX()) * MaxAngularRate * 0.5)
            ),
            new ShootSequence(() -> subwooferShotAngle, () -> subwooferShotSpeed) 
          )
        ).onFalse(new StopShoot(angleRestingPosition));

    controller.leftTrigger(0.1)
      .and(() -> !isSubwooferShot)
        .whileTrue(
          new ParallelCommandGroup(
            drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed)
                .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed)
                .withRotationalRate(rotLimiter.calculate(-controller.getLeftX()) * MaxAngularRate * 0.5)
            ),
            new ShootSequence(() -> podiumShotAngle, () -> podiumShotSpeed) 
          )
        ).onFalse(new StopShoot(angleRestingPosition));
    

    // Amp Shot
    // controller.pov(270).whileTrue(
    //   new ShootSequence(() -> 0, () -> 7)
    // ).onFalse(new StopShoot(angleRestingPosition));
    controller.pov(270).onTrue(
      new InstantCommand(this::incrementAmpMode)
    );

    new Trigger(() -> currentState.equals(ampState.PREPARED)).onTrue(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new AutoTurn(-90), 
          new AutoTurn(90), 
          () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
        ),
        angleController.setPositionCommand(0)
      )
    ).onFalse(new InstantCommand(AutoTurn::stopCommand));

    new Trigger(() -> currentState.equals(ampState.SHOOTING)).onTrue(
      new ParallelCommandGroup(
        new AutoShootSequence(
          () -> 0, 
          () -> 7, 
          angleRestingPosition
        ).andThen(new InstantCommand(this::cancelAmpMode))
      )
    );

    controller.pov(90).onTrue(
      new InstantCommand(this::cancelAmpMode)
    );

    // Trap Shot
    controller.leftBumper().whileTrue(
      new ShootSequence(() -> 0 * angleTicksPerDegree, () -> 34)
    ).onFalse(new StopShoot(angleRestingPosition));

    // controller.pov(0).whileTrue(climber.runLimitedVoltageCommand(12));
    controller.pov(0).onTrue(climber.setPositionCommand(maxClimberHeight));
    controller.pov(180).whileTrue(climber.runLimitedVoltageCommand(-12));

    controller.y().whileTrue(climber.runVoltageCommand(3));
    controller.a().whileTrue(climber.runVoltageCommand(-3));
  }

  private long timeOfLastAccess = 0;
  private double distance = 0;

  public Double[] getAngleAndSpeed() {
    System.out.println("Distance: " + distance);
    if (System.currentTimeMillis() - timeOfLastAccess < 250) {
      timeOfLastAccess = System.currentTimeMillis();
      return shooter.getAngleAndSpeed(distance);
    }

    long startingTime = System.currentTimeMillis();
    List<Double> distanceList = new ArrayList<>();
    while (System.currentTimeMillis() - startingTime < 250) {
      Double distance = limelightShooter.getDistanceFromGoal();
      if (!distance.equals(Double.NaN)) {
        try {
          if (!distance.equals(distanceList.get(distanceList.size()-1))) {
            distanceList.add(distance);
          }
        } catch (IndexOutOfBoundsException e) {
          distanceList.add(distance);
        }
        
      }
    }

    try {
      Collections.sort(distanceList);
      distanceList.remove(0);
      distanceList.remove(distanceList.size() - 1);
    } catch (IndexOutOfBoundsException e) {}

    // System.out.println("begin");
    // for (double d : distanceList) {
    //   System.out.println(d);
    // }

    distance = distanceList.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);

    timeOfLastAccess = System.currentTimeMillis();
    return shooter.getAngleAndSpeed(distance);
  }

  public double getAngle() {
    // System.out.println("Angle: " + getAngleAndSpeed()[1] * angleTicksPerDegree);
    return getAngleAndSpeed()[1];
    // return 35.75 * angleTicksPerDegree;
  }

  public double getSpeed() {
    // System.out.println("Speed: " + getAngleAndSpeed()[2]);
    return getAngleAndSpeed()[2];
    // return 80;
  }

  public void changeManualShootMode() {
    isSubwooferShot = !isSubwooferShot;
    SmartDashboard.putBoolean("Manual Shot", isSubwooferShot);
  }

  public void incrementAmpMode() {
    currentState = switch (currentState) {
      case IDLE -> ampState.PREPARED;
      case PREPARED -> ampState.SHOOTING;
      case SHOOTING -> ampState.IDLE;
      default -> ampState.IDLE;
    };
  }

  public void cancelAmpMode() {
    currentState = ampState.IDLE;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.getAutoPath("3 ring far blue");
    // return drivetrain.getAutoPath("New Auto");
  }
}
