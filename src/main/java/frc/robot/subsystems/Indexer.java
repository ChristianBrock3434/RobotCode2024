// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private TalonFX indexerMotor = new TalonFX(17);

  VelocityVoltage velocityControl;
  NeutralOut stopMode;
  
  /**
   * Creates a new Intake.
   */
  public Indexer() {
    initIntakeMotor();
  }
   
  /**
   * Initialize the intake motor
   */
  public void initIntakeMotor() {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    configs.Slot1.kP = 4; // An error of 1 rotation per second results in 2V output
    configs.Slot1.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot1.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot1.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexerMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    velocityControl = new VelocityVoltage(0, 
                                          0, 
                                          true, 
                                          0, 
                                          1, 
                                          false, 
                                          false, 
                                          false
                                          );

    stopMode = new NeutralOut();
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the intake motor
   */
  public Command runIndexerCommand(double velocity, double acceleration){
    return new Command() {
      @Override
      public void execute() {
        runIndexer(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopIndexerMotor();
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runIndexer(double velocity, double acceleration) {
    indexerMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
  }


  public Command runIndexerPercent(double speed) {
    return new Command() {
      @Override
      public void execute() {
        indexerMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        indexerMotor.set(0);
      }
    };
  }

  
  public void stopIndexerMotor() {
    indexerMotor.setControl(stopMode);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pdp.getCurrent(16));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
