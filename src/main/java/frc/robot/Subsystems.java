// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Actuation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slapper;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Subsystems {
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public static final Intake intake = new Intake(); // My intake
  public static final Actuation actuation = new Actuation(); // My actuation
  public static final Shooter shooter = new Shooter(); // My shooter
  public static final Slapper slapper = new Slapper(); // My slapper
  public static final Indexer indexer = new Indexer(); // My indexer
}
