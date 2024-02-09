// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
// import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.ShootSequence;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.StopMotors;
import frc.robot.commands.limelight.LineUpToGoal;
import frc.robot.commands.limelight.LineUpWithNotePath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

  // Lock wheels
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // public boolean intakePosition = false;
  // public boolean tuckPosition = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    linkAutoCommands();
    configureBindings();
  }

  /**
   * link commands to pathplanner for autos
   */
  public void linkAutoCommands() {
    NamedCommands.registerCommand("shoot1", new AutoShootSequence(this::getAngle, this::getSpeed));
    NamedCommands.registerCommand("shoot2", new AutoShootSequence(this::getAngle, this::getSpeed));
    NamedCommands.registerCommand("shoot3", new AutoShootSequence(this::getAngle, this::getSpeed));
    NamedCommands.registerCommand("shoot4", new AutoShootSequence(this::getAngle, this::getSpeed));
    NamedCommands.registerCommand("intake", new PickUpPiece());

    NamedCommands.registerCommand("stopMotors", new StopMotors());

    NamedCommands.registerCommand("lineUpToNote1", new LineUpWithNotePath("3 ring close blue", 0));
    NamedCommands.registerCommand("lineUpToNote2", new LineUpWithNotePath("3 ring close blue", 1));
    NamedCommands.registerCommand("lineUpToNote3", new LineUpWithNotePath("3 ring close blue", 2));

    NamedCommands.registerCommand("flashNote", limelightIntake.flashNote());
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
        drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getRightY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-controller.getRightX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-controller.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // Lock wheels on A press

    // reset the field-centric heading on start button
    controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    controller.x().onTrue(actuation.setPositionCommand(actuationPickUpPosition));
    controller.b().onTrue(actuation.setPositionCommand(actuationTuckPosition));

    // joystick.rightBumper().and(this::isIntakePosition).whileTrue(intake.runIntakeCommand(15, 40));
    controller.rightBumper().onTrue(new PickUpPiece());
    controller.leftBumper().whileTrue(intake.feedCommand(outtakeVelocity, outtakeAcceleration, 0.0));
    controller.a().whileTrue(intake.feedCommand(10, 100, 0.0));

    new Trigger(actuation::getLimitSwitch).onTrue(actuation.resetEncoderCommand());

    controller.rightTrigger(0.1).whileTrue(new ParallelCommandGroup(
      new ShootSequence(this::getAngle, this::getSpeed),
      new LineUpToGoal(10)
    )).onFalse(new StopMotors());
    // controller.leftTrigger(0.1).whileTrue(shooter.runShooterCommand(outtakeShooterVelocity, outtakeShooterAcceleration));
    // controller.leftTrigger(0.1).whileTrue(new LineUpToGoal(10));

    controller.leftTrigger(0.1).whileTrue(
      new ShootSequence(() -> 18 * angleTicksPerDegree, () -> 10)
    ).onFalse(new StopMotors());

    // joystick.y().whileTrue(new LineUpToNote());
    // controller.y().onTrue(angleController.setPositionCommand(tempAnglePosition));
    // controller.a().onTrue(angleController.setPositionCommand(angleStartingPosition));

    // joystick.getHID().setRumble(RumbleType.kBothRumble, 1);

    // controller.y().whileTrue(new InstantCommand(() -> System.out.println(DriverStation.getAlliance())));
  }

  private long timeOfLastAccess = 0;
  private double distance = 0;

  public double[] getAngleAndSpeed() {
    if (System.currentTimeMillis() - timeOfLastAccess < 250) {
      System.out.println("distance: " + distance);
      timeOfLastAccess = System.currentTimeMillis();
      return shooter.getAngleAndSpeed(distance);
    }

    long startingTime = System.currentTimeMillis();
    List<Double> distanceList = new ArrayList<>();
    while (System.currentTimeMillis() - startingTime < 250) {
      distanceList.add(limelightShooter.getDistanceFromGoal());
    }
    distance = distanceList.stream().mapToDouble(Double::doubleValue).average().orElse(-1);

    timeOfLastAccess = System.currentTimeMillis();
    return shooter.getAngleAndSpeed(distance);
  }

  public double getAngle() {
    // System.out.println("Angle: " + getAngleAndSpeed()[1] * angleTicksPerDegree);
    return getAngleAndSpeed()[1] * angleTicksPerDegree;
  }

  public double getSpeed() {
    System.out.println("Speed: " + getAngleAndSpeed()[2]);
    return getAngleAndSpeed()[2];
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.getAutoPath("3 ring close blue");
  }
}
