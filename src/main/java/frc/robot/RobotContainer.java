// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.PickUpPieceAuto;
import frc.robot.commands.automation.PrepareForShoot;
import frc.robot.commands.automation.StopIntake;
import frc.robot.commands.ShakeController;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.AutoShootSequenceNoStop;
import frc.robot.commands.automation.StopShoot;
import frc.robot.commands.automation.ZeroAngle;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.DrivePosTurning;
import frc.robot.commands.limelight.InShootingRange;
import frc.robot.commands.limelight.LineUpWithNotePath;

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
      .withDeadband(MaxSpeed * 0.13).withRotationalDeadband(MaxAngularRate * 0.1) // Add a deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

  private static boolean isSubwooferShot = true;
  private static boolean isPositionTurning = true;

  private static enum shootingState {
    IDLE,
    PREPARED,
    SHOOTING;
  };
  
  private static shootingState currentShootingState = shootingState.IDLE;

  private static enum shootingType {
    MANUAL,
    CHAIN,
    AMP,
    PASS;
  };

  private static shootingType currentShootingType = shootingType.MANUAL;

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
    ShuffleboardHandler.initDriverTab(autoChooser, () -> isSubwooferShot);
  }

  public void autoSelect() {
    // autoChooser.setDefaultOption("8-7 Blue", "3 ring far blue");
    // autoChooser.addOption("8-7 Park Blue", "3 ring far blue park");
    // autoChooser.addOption("1-2-3 Blue", "4 ring close blue");
    // autoChooser.addOption("8-6 Blue", "3 ring far blue mid");
    // // autoChooser.addOption("4-5-3-2 Blue", "5 ring close blue");

    // autoChooser.addOption("8-7 Red", "new 3 ring far red");
    // autoChooser.addOption("8-7 Park Red", "new 3 ring far red park");
    // autoChooser.addOption("1-2-3 Red", "new 4 ring close red");
    // autoChooser.addOption("8-6 Red", "new 3 ring far red mid");

    // autoChooser.addOption("Do Nothing", "Do Nothing");
    // autoChooser.addOption("4-5-3-2 Red", "5 ring close red");

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("8-7 Blue", drivetrain.getAutoPath("3 ring far blue"));
    autoChooser.addOption("8-7 Park Blue", drivetrain.getAutoPath("3 ring far blue park"));
    autoChooser.addOption("1-2-3 Blue", drivetrain.getAutoPath("4 ring close blue"));
    autoChooser.addOption("8-6 Blue", drivetrain.getAutoPath("3 ring far blue mid"));
    // autoChooser.addOption("4-5-3-2 Blue", "5 ring close blue");

    autoChooser.addOption("8-7 Red", drivetrain.getAutoPath("new 3 ring far red"));
    autoChooser.addOption("8-7 Park Red", drivetrain.getAutoPath("new 3 ring far red park"));
    autoChooser.addOption("1-2-3 Red", drivetrain.getAutoPath("new 4 ring close red"));
    // autoChooser.addOption("8-6 Red", drivetrain.getAutoPath("new 3 ring far red mid"));

    autoChooser.addOption("Do Nothing", drivetrain.getAutoPath("Do Nothing"));
  }

  /**
   * link commands to pathplanner for autos
   */
  public void linkAutoCommands() {
    NamedCommands.registerCommand("shoot", new AutoShootSequence(this::getAngle, this::getSpeed, angleRestingPosition));
    NamedCommands.registerCommand("intake", new PickUpPieceAuto(autoIntakeVoltage));
    // NamedCommands.registerCommand("intake", new PrintCommand("Intake"));

    NamedCommands.registerCommand("stopIntake", new StopIntake());
    // NamedCommands.registerCommand("tuckActuator", actuation.setPositionCommand(actuationTuckPosition));
    
    NamedCommands.registerCommand("prepareForNote", limelightIntake.prepareForNote());

    NamedCommands.registerCommand("waitForPathInterrupt", AutoTracker.waitForSignal(AutoTracker.tracked.PATH));
    NamedCommands.registerCommand("interruptPath", AutoTracker.sendSignal(AutoTracker.tracked.PATH));

    NamedCommands.registerCommand("waitForIntakeSignal", AutoTracker.waitForSignal(AutoTracker.tracked.INTAKE));
    NamedCommands.registerCommand("sendIntakeSignal", AutoTracker.sendSignal(AutoTracker.tracked.INTAKE));

    NamedCommands.registerCommand("waitForShootSignal", AutoTracker.waitForSignal(AutoTracker.tracked.SHOOTER));
    NamedCommands.registerCommand("sendShooterSignal", AutoTracker.sendSignal(AutoTracker.tracked.SHOOTER));
    
    NamedCommands.registerCommand("speedUpShooter", shooter.speedUpShooter(65, 100));

    linkShootCommands();
    linkLineUpCommands();
  }

  /**
   * link shoot commands to pathplanner
   */
  private static void linkShootCommands() {
    NamedCommands.registerCommand("shoot1CloseBlue", new AutoShootSequence(() -> 3, () -> 50, 20.5));
    NamedCommands.registerCommand("shoot2CloseBlue", new AutoShootSequence(() -> 21.5, () -> 50, 23.5));
    NamedCommands.registerCommand("shoot3CloseBlue", new AutoShootSequence(() -> 23.5, () -> 50, 24.5));
    NamedCommands.registerCommand("shoot4CloseBlue", new AutoShootSequence(() -> 24.5, () -> 50, angleRestingPosition));

    NamedCommands.registerCommand("shoot1CloseRed", new AutoShootSequence(() -> 3, () -> 50, 20));
    NamedCommands.registerCommand("shoot2CloseRed", new AutoShootSequence(() -> 21.5, () -> 50, 23.5));
    NamedCommands.registerCommand("shoot3CloseRed", new AutoShootSequence(() -> 23.5, () -> 50, 24.5));
    NamedCommands.registerCommand("shoot4CloseRed", new AutoShootSequence(() -> 24.5, () -> 50, angleRestingPosition));

    
    NamedCommands.registerCommand("shoot1CloseBlue5", new AutoShootSequence(() -> 15, () -> 65, 35));
    NamedCommands.registerCommand("shoot2CloseBlue5", new AutoShootSequence(() -> 35, () -> 65, 15));
    NamedCommands.registerCommand("shoot3CloseBlue5", new AutoShootSequenceNoStop(() -> 15, () -> 65, 22));
    NamedCommands.registerCommand("shoot4CloseBlue5", new AutoShootSequenceNoStop(() -> 22, () -> 65, 20));
    NamedCommands.registerCommand("shoot5CloseBlue5", new AutoShootSequence(() -> 20, () -> 65, angleRestingPosition));


    NamedCommands.registerCommand("shoot1FarBlue", new AutoShootSequence(() -> 18, () -> 50, 34.0));
    NamedCommands.registerCommand("shoot2FarBlue", new AutoShootSequence(() -> 34.0, () -> 65, 34.0));
    NamedCommands.registerCommand("shoot3FarBlue", new AutoShootSequence(() -> 34.0, () -> 65, angleRestingPosition));

    NamedCommands.registerCommand("shoot1FarRed", new AutoShootSequence(() -> 19.5, () -> 50, 33.75));
    NamedCommands.registerCommand("shoot2FarRed", new AutoShootSequence(() -> 34.25, () -> 65, 33.75));
    NamedCommands.registerCommand("shoot3FarRed", new AutoShootSequence(() -> 34.5, () -> 65, angleRestingPosition));
  }

  /**
   * link line up commands to pathplanner
   */
  private static void linkLineUpCommands() {
    NamedCommands.registerCommand("lineUpToNote1CloseBlue", new LineUpWithNotePath("4 ring close blue", 0, new PIDConstants(1.25), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseBlue", new LineUpWithNotePath("4 ring close blue", 1, new PIDConstants(2.0), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseBlue", new LineUpWithNotePath("4 ring close blue", 3, new PIDConstants(2.0), new PIDConstants(0.01)));

    NamedCommands.registerCommand("lineUpToNote1CloseRed", new LineUpWithNotePath("new 4 ring close red", 0, new PIDConstants(1.25), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseRed", new LineUpWithNotePath("new 4 ring close red", 1, new PIDConstants(2.0), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseRed", new LineUpWithNotePath("new 4 ring close red", 3, new PIDConstants(2.0), new PIDConstants(0.01)));


    NamedCommands.registerCommand("lineUpToNote1CloseBlue5", new LineUpWithNotePath("5 ring close blue", 0, new PIDConstants(2.0), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote2CloseBlue5", new LineUpWithNotePath("5 ring close blue", 2, new PIDConstants(2.0), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote3CloseBlue5", new LineUpWithNotePath("5 ring close blue", 4, new PIDConstants(2.0), new PIDConstants(0.01)));
    NamedCommands.registerCommand("lineUpToNote4CloseBlue5", new LineUpWithNotePath("5 ring close blue", 6, new PIDConstants(2.0), new PIDConstants(0.01)));

    
    NamedCommands.registerCommand("lineUpToNote1FarBlue", new LineUpWithNotePath("3 ring far blue", 0, new PIDConstants(2.0), new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote2FarBlue", new LineUpWithNotePath("3 ring far blue", 2, new PIDConstants(2.0), new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote3FarBlue", new LineUpWithNotePath("3 ring far blue", 4, new PIDConstants(2.0), new PIDConstants(0.1)));

    NamedCommands.registerCommand("lineUpToNote1FarRed", new LineUpWithNotePath("new 3 ring far red", 0, new PIDConstants(2.0), new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote2FarRed", new LineUpWithNotePath("new 3 ring far red", 2, new PIDConstants(2.0), new PIDConstants(0.1)));
    NamedCommands.registerCommand("lineUpToNote3FarRed", new LineUpWithNotePath("new 3 ring far red", 4, new PIDConstants(2.0), new PIDConstants(0.1)));
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
      new DrivePosTurning()
    );

    // Manual Turning Mode
    new Trigger(() -> !isPositionTurning).whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-controller.getRightY()) * MaxSpeed) 
          .withVelocityY(yLimiter.calculate(-controller.getRightX()) * MaxSpeed) 
          .withRotationalRate(rotLimiter.calculate(-controller.getLeftX()) * MaxAngularRate)
      )
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
        new InstantCommand(this::turnOnPositionTurning),
        drivetrain.runOnce(() -> drivetrain.resetOrientation()),
        new ShakeController(0.5, 0.25)
      )
    );

    // Intake
    controller.rightBumper().whileTrue(new PickUpPiece(intakeVoltage)).onFalse(new StopIntake());
    controller.b().whileTrue(new ParallelCommandGroup(
      intake.feedCommand(outtakeVelocity, outtakeAcceleration),
      indexer.runIndexerCommand(-indexerVelocity, indexerAcceleration)
    ));
    controller.x().whileTrue(new ParallelCommandGroup(
      intake.feedCommand(intakeVelocity, intakeAcceleration),
      indexer.runIndexerCommand(indexerVelocity, indexerAcceleration)
    ));

    // Chain Shot
    controller.rightTrigger(0.1).onTrue(
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.CHAIN), 
        () -> currentShootingType.equals(shootingType.CHAIN)
      )
    );

<<<<<<< Updated upstream
    new Trigger(() -> currentChainShotState.equals(chainShotState.PREPARED)).onTrue(
      new ParallelCommandGroup(
        new PrepareForShoot(
            180.0, 
            () -> chainShotAngle, 
            () -> chainShotSpeed
        ),
        new InShootingRange()
      )
=======
    new Trigger(() -> currentShootingType.equals(shootingType.CHAIN))
      .and(() -> currentShootingState.equals(shootingState.IDLE)).onTrue(
        new ParallelCommandGroup(
          new StopShoot(angleRestingPosition),
          new InstantCommand(InShootingRange::stopCommand)
        )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.CHAIN))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              180.0, 
              () -> chainShotAngle, 
              () -> chainShotSpeed
          ),
          new InShootingRange()
        )
>>>>>>> Stashed changes
    ).onFalse(new InstantCommand(AutoTurn::stopCommand));

    new Trigger(() -> currentShootingType.equals(shootingType.CHAIN))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> chainShotAngle, 
            () -> chainShotSpeed, 
            angleRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    );

    // Change Manual Shot Mode between podium and subwoofer
    controller.leftStick().onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::changeManualShootMode),
        new ShakeController(0.5, 0.25)
      )
    );

    // Manual Shoot
    controller.leftTrigger(0.1).onTrue(
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.MANUAL), 
        () -> currentShootingType.equals(shootingType.MANUAL)
      )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.MANUAL))
      .and(() -> currentShootingState.equals(shootingState.IDLE)).onTrue(
        new StopShoot(angleRestingPosition)
    );

    // Subwoofer Shot
    new Trigger(() -> currentShootingType.equals(shootingType.MANUAL))
      .and(() -> currentShootingState.equals(shootingState.PREPARED))
      .and(() -> isSubwooferShot).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              Double.NaN, 
              () -> subwooferShotAngle, 
              () -> subwooferShotSpeed
          )
        )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.MANUAL))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING))
      .and(() -> isSubwooferShot).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> subwooferShotAngle, 
            () -> subwooferShotSpeed, 
            angleRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    );

    // Podium Shot
    new Trigger(() -> currentShootingType.equals(shootingType.MANUAL))
      .and(() -> currentShootingState.equals(shootingState.PREPARED))
      .and(() -> !isSubwooferShot).onTrue(
        new ParallelCommandGroup(
          new PrepareForShoot(
              Double.NaN, 
              () -> podiumShotAngle, 
              () -> podiumShotSpeed
            )
        )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.MANUAL))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING))
      .and(() -> !isSubwooferShot).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> podiumShotAngle, 
            () -> podiumShotSpeed, 
            angleRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    );

    // Amp Shot
    controller.pov(270).onTrue(
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.AMP), 
        () -> currentShootingType.equals(shootingType.AMP)
      )
    );

<<<<<<< Updated upstream
    new Trigger(() -> currentAmpState.equals(ampState.PREPARED)).onTrue(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new AutoTurn(-90), 
          new AutoTurn(90), 
          () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
        ),
        angleController.setPositionCommand(ampAngle)
      )
=======
    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new ParallelCommandGroup(
          new ConditionalCommand(
            new AutoTurn(-90), 
            new AutoTurn(90), 
            () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
          ),
          angleController.setPositionCommand(ampAngle)
        )
>>>>>>> Stashed changes
    ).onFalse(new InstantCommand(AutoTurn::stopCommand));

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> ampAngle, 
            () -> ampSpeed, 
            angleRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    );

    // Pass Shot
    controller.leftBumper().onTrue(
<<<<<<< Updated upstream
      new InstantCommand(this::incrementPassMode)
    );

    new Trigger(() -> currentPassState.equals(passState.IDLE)).onTrue(
      new StopShoot(angleRestingPosition)
    );

    new Trigger(() -> currentPassState.equals(passState.PREPARED)).onTrue(
<<<<<<< HEAD
      new PrepareForShoot(180.0, passShotAngle, passShotSpeed)
=======
      new ConditionalCommand(
        new InstantCommand(this::incrementShootingMode), 
        setShootingTypeCommand(shootingType.PASS), 
        () -> currentShootingType.equals(shootingType.PASS)
      )
    );

    new Trigger(() -> currentShootingType.equals(shootingType.PASS))
      .and(() -> currentShootingState.equals(shootingState.IDLE)).onTrue(
        new StopShoot(angleRestingPosition)
    );

    new Trigger(() -> currentShootingType.equals(shootingType.PASS))
      .and(() -> currentShootingState.equals(shootingState.PREPARED)).onTrue(
        new PrepareForShoot(
            180.0, 
            () -> passShotAngle, 
            () -> passShotSpeed
        )
>>>>>>> Stashed changes
=======
      new PrepareForShoot(
          180.0, 
          () -> passShotAngle, 
          () -> passShotSpeed
      )
>>>>>>> 200cc962a6af272414ce912c1b359dc6c3c90229
    ).onFalse(new InstantCommand(AutoTurn::stopCommand));

    new Trigger(() -> currentShootingType.equals(shootingType.PASS))
      .and(() -> currentShootingState.equals(shootingState.SHOOTING)).onTrue(
        new ParallelCommandGroup(
          new AutoShootSequence(
            () -> passShotAngle, 
            () -> passShotSpeed, 
            angleRestingPosition
          ).andThen(new InstantCommand(this::stopShooting))
        )
    );

    // Cancel all current Modes
    controller.pov(90).onTrue(
      new ParallelCommandGroup(
        new InstantCommand(this::stopShooting)
      )
    );

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

    controller.y().whileTrue(climber.runVoltageCommand(3));
    controller.a().whileTrue(climber.runVoltageCommand(-3));

    // Reset angle
    controller.back().onTrue(new ZeroAngle());

    new Trigger(() -> angleController.getZeroSensor()).onTrue(new InstantCommand(angleController::zeroOnSensor));
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
   * Change the manual shot mode between subwoofer and podium
   */
  public void changeManualShootMode() {
    isSubwooferShot = !isSubwooferShot;
  }

  public void incrementShootingMode() {
    currentShootingState = switch (currentShootingState) {
      case IDLE -> shootingState.PREPARED;
      case PREPARED -> shootingState.SHOOTING;
      case SHOOTING -> shootingState.IDLE;
      default -> shootingState.IDLE;
    };
  }

  public void stopShooting() {
    currentShootingState = shootingState.IDLE;
  }

  public void setShootingType(shootingType type) {
    currentShootingType = type;
    currentShootingState = shootingState.PREPARED;
  }

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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drivetrain.getAutoPath(autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
