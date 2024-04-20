// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.SlapperConstants.*;
import static frc.robot.Subsystems.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.core.sym.Name;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.PickUpPieceAuto;
import frc.robot.commands.automation.PrepareForShoot;
import frc.robot.commands.automation.ShootSequence;
import frc.robot.commands.automation.StopIntake;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ShakeController;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.AutoShootSequenceNoStop;
import frc.robot.commands.automation.StopShoot;
import frc.robot.commands.automation.ZeroAngle;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.AutoTurnToGoal;
import frc.robot.commands.drivetrain.DrivePosTurning;
import frc.robot.commands.limelight.InShootingRange;
import frc.robot.commands.limelight.LineUpWithNotePath;
import frc.robot.commands.limelight.SeedPoseEstimation;
import frc.robot.commands.limelight.SeedPoseOnce;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // private static SendableChooser<String> autoChooser = new SendableChooser<>();
  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Artificial Acceleration for less sliding and more control
  private static SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private static SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private static SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
      .withDeadband(MaxSpeed * 0.13).withRotationalDeadband(MaxAngularRate * 0.13) // Add a deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

  private static boolean isAutoLineUp = true;
  private static boolean isPositionTurning = false;

  private static enum shootingState {
    IDLE,
    PREPARED,
    SHOOTING;
  };
  
  private static shootingState currentShootingState = shootingState.IDLE;

  private static enum shootingType {
    SUBWOOFER,
    PODIUM,
    CHAIN,
    CHAMPIONSHIP,
    AMP,
    PASS;
  };

  private static shootingType currentShootingType = shootingType.SUBWOOFER;

  // Lock wheels
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // private final SwerveRequest.PointWheelsAt pointForward = new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    linkAutoCommands();
    configureBindings();
    autoSelect();
    ShuffleboardHandler.initSensorTab();
    ShuffleboardHandler.initDriverTab(autoChooser, () -> isAutoLineUp);
    ShuffleboardHandler.initTuningTab();
  }

  /**
   * Selects the autonomous mode based on user input.
   * This method sets up the options for the autonomous mode chooser and assigns the corresponding auto paths to each option.
   */
  public void autoSelect() {
    // autoChooser.setDefaultOption("8-7 Blue", "3 ring far blue");
    // autoChooser.addOption("8-7 Park Blue", "3 ring far blue park");
    // autoChooser.addOption("1-2-3 Blue", "4 ring close blue");
    // autoChooser.addOption("8-6 Blue", "3 ring far blue mid");
    // // autoChooser.addOption("4-5-3-2 Blue", "5 ring close blue");

    // autoChooser.addOption("8-7 Red", "new 3 ring far red");
    // autoChooser.addOption("8-7 Park Red", "new 3 ring far red park");
    // autoChooser.addOption("1-2-3 Red", "4 ring close red");
    // autoChooser.addOption("8-6 Red", "new 3 ring far red mid");

    // autoChooser.addOption("Do Nothing", "Do Nothing");
    // autoChooser.addOption("4-5-3-2 Red", "5 ring close red");

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("8-7 Blue", drivetrain.getAutoPath("3 ring far blue"));
    autoChooser.addOption("8-7 Park Blue", drivetrain.getAutoPath("3 ring far blue park"));
    autoChooser.addOption("1-2-3 Blue", drivetrain.getAutoPath("4 ring close blue"));
    // autoChooser.addOption("8-6 Blue", drivetrain.getAutoPath("3 ring far blue mid"));
    autoChooser.addOption("7-8-6 Blue", drivetrain.getAutoPath("7-8-6 far blue"));
    // autoChooser.addOption("4-5-3-2 Blue", "5 ring close blue");

    autoChooser.addOption("8-7 Red", drivetrain.getAutoPath("new 3 ring far red"));
    autoChooser.addOption("8-7 Park Red", drivetrain.getAutoPath("new 3 ring far red park"));
    autoChooser.addOption("1-2-3 Red", drivetrain.getAutoPath("4 ring close red"));
    // autoChooser.addOption("8-6 Red", drivetrain.getAutoPath("new 3 ring far red mid"));
    autoChooser.addOption("7-8-6 Red", drivetrain.getAutoPath("7-8-6 far red"));

    autoChooser.addOption("Do Nothing", drivetrain.getAutoPath("Do Nothing"));
  }

  /**
   * link commands to pathplanner for autos
   */
  public void linkAutoCommands() {
    NamedCommands.registerCommand("shoot", new AutoShootSequence(this::getAngle, this::getSpeed, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("intake", new PickUpPieceAuto(autoIntakeVoltage));
    // NamedCommands.registerCommand("intake", new PrintCommand("Intake"));

    NamedCommands.registerCommand("stopIntake", new StopIntake());
    // NamedCommands.registerCommand("tuckActuator", actuation.setPositionCommand(actuationTuckPosition));
    
    NamedCommands.registerCommand("prepareForNote", limelightIntake.prepareForNote());
    NamedCommands.registerCommand("Seed Pose", new SeedPoseOnce());
    NamedCommands.registerCommand("Seed Pose Continous", new SeedPoseEstimation());

    NamedCommands.registerCommand("waitForPathInterrupt", AutoTracker.waitForSignal(AutoTracker.tracked.PATH));
    NamedCommands.registerCommand("interruptPath", AutoTracker.sendSignal(AutoTracker.tracked.PATH));

    NamedCommands.registerCommand("waitForIntakeSignal", AutoTracker.waitForSignal(AutoTracker.tracked.INTAKE));
    NamedCommands.registerCommand("sendIntakeSignal", AutoTracker.sendSignal(AutoTracker.tracked.INTAKE));

    NamedCommands.registerCommand("waitForShootSignal", AutoTracker.waitForSignal(AutoTracker.tracked.SHOOTER));
    NamedCommands.registerCommand("sendShooterSignal", AutoTracker.sendSignal(AutoTracker.tracked.SHOOTER));
    
    NamedCommands.registerCommand("speedUpShooter", shooter.speedUpShooter(50, 100));
    NamedCommands.registerCommand("speedUpShooter55", shooter.speedUpShooter(55, 100));
    NamedCommands.registerCommand("SetAngleRest", angleController.setPositionCommand(angleRestingPosition));

    NamedCommands.registerCommand("SetAngleShootChain", angleController.setPositionCommand(31.5));

    NamedCommands.registerCommand("zeroAngle", new ZeroAngle());

    NamedCommands.registerCommand("Auto Aim Champion", new AutoTurnToGoal(() -> championshipShotOffset).withTimeout(0.25));

    NamedCommands.registerCommand("prepareSlapper", slapper.setPositionCommand(slapperRestingPosition));

    linkShootCommands();
    linkLineUpCommands();
  }

  /**
   * link shoot commands to pathplanner
   */
  private static void linkShootCommands() {
    NamedCommands.registerCommand("shoot1CloseBlue", new AutoShootSequence(() -> 3, () -> 35, 22.0, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot2CloseBlue", new AutoShootSequence(() -> 22.0, () -> 40, 22.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot3CloseBlue", new AutoShootSequence(() -> 22.5, () -> 40, 22.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot4CloseBlue", new AutoShootSequence(() -> 22.5, () -> 40, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));

    NamedCommands.registerCommand("shoot1CloseRed", new AutoShootSequence(() -> 3, () -> 35, 22.0, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot2CloseRed", new AutoShootSequence(() -> 22.0, () -> 40, 22.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot3CloseRed", new AutoShootSequence(() -> 22.5, () -> 40, 22.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot4CloseRed", new AutoShootSequence(() -> 22.5, () -> 30, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));

    
    NamedCommands.registerCommand("shoot1CloseBlue5", new AutoShootSequence(() -> 15, () -> 50, 35, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot2CloseBlue5", new AutoShootSequence(() -> 35, () -> 50, 15, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot3CloseBlue5", new AutoShootSequenceNoStop(() -> 15, () -> 50, 22, () -> slapperRestingPosition));
    NamedCommands.registerCommand("shoot4CloseBlue5", new AutoShootSequenceNoStop(() -> 22, () -> 50, 20, () -> slapperRestingPosition));
    NamedCommands.registerCommand("shoot5CloseBlue5", new AutoShootSequence(() -> 20, () -> 50, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));


    NamedCommands.registerCommand("shoot1FarBlue", new AutoShootSequence(() -> 19.5, () -> 40, 5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot2FarBlue", new AutoShootSequence(() -> 31.75, () -> 50, 5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot3FarBlue", new AutoShootSequence(() -> 31.75, () -> 50, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));

    NamedCommands.registerCommand("shoot1FarRed", new AutoShootSequence(() -> 20, () -> 40, 5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot2FarRed", new AutoShootSequence(() -> 31.75, () -> 50, 5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot3FarRed", new AutoShootSequence(() -> 31.5, () -> 50, angleRestingPosition, () -> slapperRestingPosition, slapperRestingPosition));

    NamedCommands.registerCommand("shoot0blue", new AutoShootSequence(() -> 18.5, () -> 40, 33, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot7blue", new AutoShootSequence(() -> 33.5, () -> 50, 33, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot8blue", new AutoShootSequence(() -> 33.5, () -> 50, 5, () -> slapperRestingPosition, slapperRestingPosition));

    NamedCommands.registerCommand("shoot0red", new AutoShootSequence(() -> 18.5, () -> 40, 33, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot7red", new AutoShootSequence(() -> 33.5, () -> 50, 33, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand("shoot8red", new AutoShootSequence(() -> 33.5, () -> 50, 5, () -> slapperRestingPosition, slapperRestingPosition));
  }

  /**
   * link line up commands to pathplanner
   */
  private static void linkLineUpCommands() {
    NamedCommands.registerCommand("lineUpToNote1CloseBlue", new LineUpWithNotePath("4 ring close blue", 0, new PIDConstants(2.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseBlue", new LineUpWithNotePath("4 ring close blue", 1, new PIDConstants(2.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseBlue", new LineUpWithNotePath("4 ring close blue", 3, new PIDConstants(2.0), 4, new PIDConstants(0.01)));

    NamedCommands.registerCommand("lineUpToNote1CloseRed", new LineUpWithNotePath("4 ring close red", 0, new PIDConstants(2.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseRed", new LineUpWithNotePath("4 ring close red", 1, new PIDConstants(2.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseRed", new LineUpWithNotePath("4 ring close red", 3, new PIDConstants(2.0), 4, new PIDConstants(0.01)));


    NamedCommands.registerCommand("lineUpToNote1CloseBlue5", new LineUpWithNotePath("5 ring close blue", 0, new PIDConstants(4.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseBlue5", new LineUpWithNotePath("5 ring close blue", 2, new PIDConstants(4.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseBlue5", new LineUpWithNotePath("5 ring close blue", 4, new PIDConstants(2.0), 4, new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote4CloseBlue5", new LineUpWithNotePath("5 ring close blue", 6, new PIDConstants(2.0), 4, new PIDConstants(0.01)));

    
    NamedCommands.registerCommand("lineUpToNote1FarBlue", new LineUpWithNotePath("3 ring far blue", 0, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote2FarBlue", new LineUpWithNotePath("3 ring far blue", 2, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote3FarBlue", new LineUpWithNotePath("3 ring far blue", 4, new PIDConstants(4.0), 4, new PIDConstants(0.1)));

    NamedCommands.registerCommand("lineUpToNote1FarRed", new LineUpWithNotePath("new 3 ring far red", 0, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote2FarRed", new LineUpWithNotePath("new 3 ring far red", 2, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote3FarRed", new LineUpWithNotePath("new 3 ring far red", 4, new PIDConstants(4.0), 4, new PIDConstants(0.1)));

    NamedCommands.registerCommand("lineUpToNote7Blue", new LineUpWithNotePath("7-8-6 far blue", 0, new PIDConstants(4.0), 4, new PIDConstants(0.05)));
    NamedCommands.registerCommand("lineUpToNote8Blue", new LineUpWithNotePath("7-8-6 far blue", 2, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote6Blue", new LineUpWithNotePath("7-8-6 far blue", 4, new PIDConstants(4.0), 4, new PIDConstants(0.1)));

    NamedCommands.registerCommand("lineUpToNote7Red", new LineUpWithNotePath("7-8-6 far red", 0, new PIDConstants(4.0), 4, new PIDConstants(0.05)));
    NamedCommands.registerCommand("lineUpToNote8Red", new LineUpWithNotePath("7-8-6 far red", 2, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote6Red", new LineUpWithNotePath("7-8-6 far red", 4, new PIDConstants(4.0), 4, new PIDConstants(0.1)));
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
    drivetrain.setDefaultCommand(
      driveCommand()
    );

    // Manual Turning Mode
    new Trigger(() -> isPositionTurning).whileTrue(
      new DrivePosTurning()
    );

    // Change Turning Mode
    controller.rightStick().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::changeTurningMode),
        new ShakeController(0.5, 0.25)
      )
    );

    // reset the field-centric heading on start button    
    controller.start().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::turnOffPositionTurning),
        drivetrain.runOnce(() -> drivetrain.resetOrientation()),
        new ShakeController(0.5, 0.25)
      )
    );

    // Intake
    controller.rightBumper().whileTrue(new PickUpPiece(intakeVoltage)).onFalse(new StopIntake());
    controller.b().whileTrue(new ParallelCommandGroup(
      intake.runVoltageCommand(-4),
      indexer.runIndexerCommand(-indexerVelocity, indexerAcceleration)
    )).onFalse(intake.stopIntakeCommand());
    controller.x().whileTrue(new ParallelCommandGroup(
      intake.runVoltageCommand(4),
      indexer.runIndexerCommand(indexerVelocity, indexerAcceleration)
    )).onFalse(intake.stopIntakeCommand());

    // On Stop Shooting
    new Trigger(() -> currentShootingState.equals(shootingState.IDLE)).onTrue(
      new ParallelCommandGroup(
        new StopShoot(angleRestingPosition, slapperRestingPosition),
        new InstantCommand(InShootingRange::stopCommand)
      )
    );

    // new Trigger(() -> true).whileTrue(this.shootRightTriggerCommand());
    // shootRightTriggerCommand().schedule();

    // new Trigger(() -> true).whileTrue(this.shootLeftTriggerCommand());
    // shootLeftTriggerCommand().schedule();

    // Chain Shot
    new Trigger(() -> currentShootingType.equals(shootingType.CHAIN))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              () -> chainShotAngle, 
              () -> chainShotSpeed,
              () -> slapperRestingPosition
          ),
          new InShootingRange()
        )
    // ).onFalse(new InstantCommand(AutoTurnToGoal::stopCommand));
    );

    new Trigger(() -> currentShootingType.equals(shootingType.CHAIN))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new SequentialCommandGroup(
          new ConditionalCommand(
            new AutoTurnToGoal(() -> chainShotOffset).withTimeout(0.75), 
            new ConditionalCommand(
              new AutoTurn(() -> chainShotManualRot).withTimeout(0.5), 
              new AutoTurn(() -> -chainShotManualRot).withTimeout(0.5), 
              this::isBlueSide),
            () -> isAutoLineUp
          ),
          new AutoShootSequence(
            () -> chainShotAngle, 
            () -> chainShotSpeed, 
            angleRestingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    ).onFalse(driveCommand());

    // Championship Shot
    new Trigger(() -> currentShootingType.equals(shootingType.CHAMPIONSHIP))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              () -> championshipShotAngle, 
              () -> championshipShotSpeed,
              () -> slapperRestingPosition
          ),
          new InShootingRange()
        )
    // ).onFalse(new InstantCommand(AutoTurnToGoal::stopCommand));
    );

    new Trigger(() -> currentShootingType.equals(shootingType.CHAMPIONSHIP))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new SequentialCommandGroup(
          new ConditionalCommand(
            new AutoTurnToGoal(() -> championshipShotOffset).withTimeout(0.75), 
            new ConditionalCommand(
              new AutoTurn(() -> championshipShotManualRot).withTimeout(0.5), 
              new AutoTurn(() -> -championshipShotManualRot).withTimeout(0.5), 
              this::isBlueSide),
            () -> isAutoLineUp
          ),
          new AutoShootSequence(
            () -> championshipShotAngle, 
            () -> championshipShotSpeed, 
            angleRestingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    ).onFalse(driveCommand());

    // Change Manual Shot Mode between podium and subwoofer
    controller.leftStick().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::changeAutoLineUpMode),
        new ShakeController(0.5, 0.25)
      )
    );

    // Subwoofer Shot
    new Trigger(() -> currentShootingType.equals(shootingType.SUBWOOFER))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              () -> subwooferShotAngle, 
              () -> subwooferShotSpeed,
              () -> slapperRestingPosition
          )
        )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.SUBWOOFER))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new SequentialCommandGroup(
          // new AutoTurnToGoal(),
          new AutoShootSequence(
            () -> subwooferShotAngle, 
            () -> subwooferShotSpeed, 
            angleRestingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    ).onFalse(driveCommand());

    // Podium Shot
    new Trigger(() -> currentShootingType.equals(shootingType.PODIUM))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              () -> podiumShotAngle, 
              () -> podiumShotSpeed,
              () -> slapperRestingPosition
            )
        )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.PODIUM))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new SequentialCommandGroup(
          new ConditionalCommand(
            new AutoTurnToGoal(() -> podiumShotOffset).withTimeout(0.75), 
            new ConditionalCommand(
              new AutoTurn(() -> podiumShotManualRot).withTimeout(0.5), 
              new AutoTurn(() -> -podiumShotManualRot).withTimeout(0.5), 
              this::isBlueSide),
            () -> isAutoLineUp
          ),
          new AutoShootSequence(
            () -> podiumShotAngle, 
            () -> podiumShotSpeed, 
            angleRestingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    ).onFalse(driveCommand());

    // Amp Shot
    controller.pov(270).onTrue(
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.AMP), 
        () -> currentShootingType.equals(shootingType.AMP)
      )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new ConditionalCommand(
            new AutoTurn(-90), 
            new AutoTurn(90), 
            () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
          ),
          angleController.setPositionCommandSupplier(() -> ampAngle),
          slapper.setPositionCommand(slapperAmpPosition)
        )
    ).onFalse(driveCommand());

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> ampAngle, 
            () -> ampSpeed, 
            angleRestingPosition,
            () -> slapperAmpPosition,
            slapperPushNotePosition
          ).andThen(new SequentialCommandGroup(
            slapper.setPositionCommand(slapperPostAmpPosition),
            new WaitCommand(0.75),
            new InstantCommand(this::stopShooting)
          ).beforeStarting(slapper.waitUntilAtPosition(slapperPushNotePosition)))
        )
    );

    // Pass Shot
    controller.leftBumper().onTrue(
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.PASS), 
        () -> currentShootingType.equals(shootingType.PASS)
      )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.PASS))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        // new ParallelCommandGroup(
        new PrepareForShoot(
          () -> passShotAngle, 
          () -> passShotSpeed,
          () -> slapperRestingPosition
        )
          // new ConditionalCommand(
          //   new ConditionalCommand(
          //     new AutoTurn(() -> passShotManualRot), 
          //     new AutoTurn(() -> -passShotManualRot), 
          //     this::isBlueSide
          //   ),
          //   driveCommand(),
          //   () -> isAutoLineUp
          // )
        // )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.PASS))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new SequentialCommandGroup(
          new AutoShootSequence(
            () -> passShotAngle, 
            () -> passShotSpeed, 
            angleRestingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    ).onFalse(driveCommand());

    // Cancel all current Modes
    controller.pov(90).onTrue(
      new InstantCommand(this::stopShooting)
    );

    // Change the angle of the recent shot to shoot higher
    controller.y().onTrue(new InstantCommand(() -> changeRecentShotAngle(-0.25)));

    // Change the angle of the recent shot to shoot lower
    controller.a().onTrue(new InstantCommand(() -> changeRecentShotAngle(0.25)));

    // controller.y().whileTrue(slapper.setPositionCommand(slapperAmpPosition));
    // controller.a().whileTrue(slapper.setPositionCommand(slapperRestingPosition));

    // Trap Shot
    // controller.leftBumper().whileTrue(
    //   new ShootSequence(() -> 0 * angleTicksPerDegree, () -> 34)
    // ).onFalse(new StopShoot(angleRestingPosition));
    // controller.leftBumper().onTrue(
    //   new InstantCommand(this::incrementTrapMode)
    // );

    // new Trigger(() -> currentTrapState.equals(trapState.PREPARED)).onTrue(
    //   new ParallelCommandGroup(
    //     new LineUpToTrap(),
    //     angleController.setPositionCommand(0)
    //   )
    // ).onFalse(new InstantCommand(LineUpToTrap::stopCommand));

    // new Trigger(() -> currentTrapState.equals(trapState.SHOOTING)).onTrue(
    //   new ParallelCommandGroup(
    //     new ShootTrap().andThen(new InstantCommand(this::cancelTrapMode))
    //   )
    // );

    // Climber
    controller.pov(0).whileTrue(climber.runLimitedVoltageCommand(12));
    controller.pov(180).whileTrue(climber.runLimitedVoltageCommand(-12));

    // controller.y().whileTrue(climber.runVoltageCommand(3));
    // controller.a().whileTrue(climber.runVoltageCommand(-3));

    // Reset angle
    controller.back().onTrue(new ZeroAngle());

    new Trigger(() -> angleController.getZeroSensor()).onTrue(new InstantCommand(angleController::zeroOnSensor));

    // controller.rightTrigger(0.1).whileTrue(shooter.runShooterCommand(50, 100));
  }

  public Command driveCommand() {
    return drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getHID().getRightY()) * MaxSpeed) 
          .withVelocityY(yLimiter.calculate(-controller.getHID().getRightX()) * MaxSpeed) 
          .withRotationalRate(rotLimiter.calculate(-controller.getHID().getLeftX()) * MaxAngularRate));
  }

  private long timeOfLastAccess = 0;
  private double distance = 0;

  /**
   * Get the angle and speed for the shooter based on the distance from the goal
   * @return the angle and speed for the shooter
   */
  public Double[] getAngleAndSpeed() {
    if (System.currentTimeMillis() - timeOfLastAccess < 250) {
      timeOfLastAccess = System.currentTimeMillis();
      return shooter.getAngleAndSpeed(distance);
    }

    long startingTime = System.currentTimeMillis();
    List<Double> distanceList = new ArrayList<>();
    while (System.currentTimeMillis() - startingTime < 250) {
      Double distance = drivetrain.getDistanceFromGoal();
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

    distance = distanceList.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);

    timeOfLastAccess = System.currentTimeMillis();
    return shooter.getAngleAndSpeed(distance);
  }

  /**
   * Get the angle for the shooter based on the distance from the goal
   * @return the angle for the shooter
   */
  public double getAngle() {
    return getAngleAndSpeed()[1];
  }

  /**
   * Get the speed for the shooter based on the distance from the goal
   * @return the speed for the shooter
   */
  public double getSpeed() {
    return getAngleAndSpeed()[2];
  }
  
  /**
   * Change the turning mode between position turning and normal turning
   */
  public void changeTurningMode() {
    isPositionTurning = !isPositionTurning;
  }

  /**
   * Turn on position turning mode
   */
  public void turnOnPositionTurning() {
    isPositionTurning = true;
  }

  /**
   * Turn off position turning mode
   */
  public void turnOffPositionTurning() {
    isPositionTurning = false;
  }

  /**
   * Change from auto line up to auto turn
   */
  public void changeAutoLineUpMode() {
    isAutoLineUp = !isAutoLineUp;
  }

  /**
   * Increments the shooting mode to the next state.
   * The shooting mode follows the sequence: IDLE -> PREPARED -> SHOOTING -> IDLE.
   */
  public void incrementShootingMode() {
    currentShootingState = switch (currentShootingState) {
      case IDLE -> shootingState.PREPARED;
      case PREPARED -> shootingState.SHOOTING;
      case SHOOTING -> shootingState.IDLE;
      default -> shootingState.IDLE;
    };
  }

  /**
   * sets the shooting state to the IDLE.
   */
  public void stopShooting() {
    currentShootingState = shootingState.IDLE;
  }

  /**
   * Sets the shooting type for the robot.
   * 
   * @param type The shooting type to set.
   */
  public void setShootingType(shootingType type) {
    currentShootingType = type;
    currentShootingState = shootingState.PREPARED;
  }

  /**
   * Sets the shooting type command.
   *
   * @param type the shooting type to set
   * @return the command object that sets the shooting type
   */
  public Command setShootingTypeCommand(shootingType type) {
    return new Command() {
      @Override
      public void execute() {
        setShootingType(type);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Changes the angle of the recent shot based on the current shooting type.
   * The amount parameter specifies the amount by which the angle should be changed.
   *
   * @param amount the amount by which the angle should be changed
   */
  public void changeRecentShotAngle(double amount) {
    switch (currentShootingType) {
      case AMP:
        ampAngle += amount;
        break;
      
      case SUBWOOFER:
        subwooferShotAngle += amount;  
        break;

      case PODIUM:
        podiumShotAngle += amount;
        break;

      case CHAIN:
        chainShotAngle += amount;
        break;

      case CHAMPIONSHIP:
        championshipShotAngle += amount;
        break;

      case PASS:
        passShotAngle += amount;
        break;
    }
  }

  private double maxRightPressedDown = 0;
  /**
   * Handles the shooting action when the right trigger is pressed.
   */
  public void shootRightTrigger() {
    double rightTrigger = controller.getHID().getRightTriggerAxis();

    if (currentShootingState == shootingState.IDLE) {
      maxRightPressedDown = Math.max(maxRightPressedDown, rightTrigger);

      if (maxRightPressedDown > 0.1 && rightTrigger < 0.1) {
        if (maxRightPressedDown < 0.9) {
          setShootingType(shootingType.CHAMPIONSHIP);
        } else {
          setShootingType(shootingType.SUBWOOFER);
        }
        maxRightPressedDown = 0;
      }
    } 
    else if (currentShootingState == shootingState.PREPARED) {
      if (rightTrigger > 0.1) {
        currentShootingState = shootingState.SHOOTING;
      }
    }
  }

  private double maxLeftPressedDown = 0;
  /**
   * Handles the shooting action when the left trigger is pressed.
   */
  public void shootLeftTrigger() {
    double leftTrigger = controller.getHID().getLeftTriggerAxis();

    if (currentShootingState == shootingState.IDLE) {
      maxLeftPressedDown = Math.max(maxLeftPressedDown, leftTrigger);

      if (maxLeftPressedDown > 0.1 && leftTrigger < 0.1) {
        if (maxLeftPressedDown < 0.9) {
          setShootingType(shootingType.CHAIN);
        } else {
          setShootingType(shootingType.PODIUM);
        }
        maxLeftPressedDown = 0;
      }
    } 
    else if (currentShootingState == shootingState.PREPARED) {
      if (leftTrigger > 0.1) {
        currentShootingState = shootingState.SHOOTING;
      }
    }
  }

  public Boolean isBlueSide() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      System.out.println("The Alliance is empty, Please Select an Alliance");
    } else if (alliance.get().equals(Alliance.Red)) {
      return false;
    } else if (alliance.get().equals(Alliance.Blue)) {
      return true;
    }
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drivetrain.getAutoPath(autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
