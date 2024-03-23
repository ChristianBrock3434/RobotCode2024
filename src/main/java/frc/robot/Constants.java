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

  public static double farShotDistance = 4.4; // meters

  public final class ActuationConstants {
    public static final double actuationGearRatio = 8;

    // 360 degrees per rotation, 8:1 gear ratio
    public static final double actuationTicksPerDegree = 1.0 / 360.0 * actuationGearRatio;

    public static final double actuationStartPosition = -65;
    public static double actuationPickUpPosition = 100;
    public static final double actuationTuckPosition = -65;
  }

  public final class IntakeConstants {
    public static final double intakeGearRatio = 1.3333333333333333333;
    
    public static final double intakeVoltage = 7;
    public static final double autoIntakeVoltage = 5;

    public static final double feedVelocity = 60;
    public static final double feedAcceleration = 102;

    public static final double intakeVelocity = 15;
    public static final double intakeAcceleration = 40;

    public static final double outtakeVelocity = -15;
    public static final double outtakeAcceleration = 40;
  }

  public final class ShooterConstants {
    public static final double shooterSequenceAcceleration = 100;

    public static final double trapSpeed = 36;
    public static double ampSpeed = 7.2;

    public static double subwooferShotSpeed = 40;
    public static double podiumShotSpeed = 50; //55 Half Line Shot 62
    public static double chainShotSpeed = 46.5;
    public static final double passShotSpeed = 40;
  }

  public final class IndexerConstants {
    public static final double indexerVelocity = 60; //40 amp //60 shooting
    public static final double indexerAcceleration = 100;
  }

  public final class AngleControllerConstants {
    public static final double angleTicksPerDegree = (11.78 * 1.75 / 360 * 4) / 1.5; //Estimate given by Adam, ask him how to do it if you need

    public static final double angleStartingPosition = 0;
    public static final double angleRestingPosition = 25.75;

    public static final double trapAngle = 0;
    public static double ampAngle = -6;
    
    public static double subwooferShotAngle = 0.5;
    public static double podiumShotAngle = 24.25; //23 Half Line Shot 34.75
    public static double chainShotAngle = 35.25;
    public static double passShotAngle = 15;
  }

  public final class ClimberConstants {
    public static final double minClimberHeight = 0;
    public static final double maxClimberHeight = 240;
  }

  public final class FieldConstants {
    public static final double blueSpeakerX = 0;
    public static final double blueSpeakerY = 5.5696358;

    public static final double redSpeakerX = 0;
    public static final double redSpeakerY = 5.5696358;
  }
  
}
