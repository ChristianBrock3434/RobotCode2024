// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double MaxSpeed = 6; // 6 meters per second desired top speed
  public static final double MaxAngularRate = Math.PI * 4; // Half a rotation per second max angular velocity
  
  public static final CommandXboxController controller = new CommandXboxController(0); // My joystick

  public final class ActuationConstants {
    public static final double actuationGearRatio = 8;

    // 360 degrees per rotation, 8:1 gear ratio
    public static final double actuationTicksPerDegree = 1.0 / 360.0 * actuationGearRatio;

    public static final double actuationStartPosition = -65 * actuationTicksPerDegree;
    public static final double actuationPickUpPosition = 100 * actuationTicksPerDegree;
    public static final double actuationTuckPosition = -65 * actuationTicksPerDegree;
  }

  public final class IntakeConstants {
    public static final double intakeGearRatio = 1.3333333333333333333;
    
    public static final double intakeVoltage = 5;
    public static final double autoIntakeVoltage = 4;

    public static final double feedVelocity = 60;
    public static final double feedAcceleration = 102;

    public static final double outtakeVelocity = -15;
    public static final double outtakeAcceleration = 40;
  }

  public final class ShooterConstants {
    public static final double shooterSequenceVelocity = 40; //45 //60 //10 for amp
    public static final double shooterSequenceAcceleration = 100;

    public static final double outtakeShooterVelocity = -40;
    public static final double outtakeShooterAcceleration = 100;
  }

  public final class IndexerConstants {
    public static final double indexerVelocity = 60; //40 amp //60 shooting
    public static final double indexerAcceleration = 100;
  }

  public final class AngleControllerConstants {
    public static final double angleTicksPerDegree = (11.78 / 360 * 4) / 1.5; //Estimate given by Adam, ask him how to do it if you need

    public static final double angleStartingPosition = 0 * angleTicksPerDegree;
    public static final double angleRestingPosition = 5 * angleTicksPerDegree;
  }

  public final class ClimberConstants {
    public static final double minClimberHeight = 0;
    public static final double maxClimberHeight = 210;
  }

  public final class FieldConstants {
    public static final double blueSpeakerX = -8.3;
    public static final double blueSpeakerY = 1.5;

    public static final double redSpeakerX = 0;
    public static final double redSpeakerY = 0;
  }
  
}
