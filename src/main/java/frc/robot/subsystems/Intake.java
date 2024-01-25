// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(138657);
  private TalonFX actuationMotor = new TalonFX(36412);

  VelocityVoltage velocityControl;
  MotionMagicVoltage motionMagicControl;
  
  /**
   * Creates a new Intake.
   */
  public Intake() {
    initIntakeMotor();
    initActuationMotor();
  }
   
  /**
   * Initialize the intake motor
   */
  public void initIntakeMotor() {
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    velocityControl = new VelocityVoltage(0, 
                                          0, 
                                          true, 
                                          0, 
                                          0, 
                                          false, 
                                          false, 
                                          false
                                        );
  }

  /**
   * Initialize the actuation motor
   */
  public void initActuationMotor() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = actuationMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    motionMagicControl = new MotionMagicVoltage(0, 
                                                true, 
                                                0, 
                                                0, 
                                                false, 
                                                false, 
                                                false
                                              );
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the intake motor
   */
  public Command runIntakeCommand(double velocity, double acceleration){
    return new Command() {
      @Override
      public void execute() {
        runIntakeMotor(velocity, acceleration);
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runIntakeMotor(double velocity, double acceleration) {
    intakeMotor.setControl(velocityControl
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
  }

  /**
   * Run the actuation motor to a given position
   * @param position in encoder value
   * @return a command that will run the actuation motor
   */
  public Command setPositionCommand(double position) {
    return new Command() {
      @Override
      public void execute() {
        setPosition(position);
      }
    };
  }

  /**
   * Run the actuation motor to a given position
   * @param position in encoder value
   */
  public void setPosition(double position) {
    actuationMotor.setControl(motionMagicControl
                              .withPosition(position)
                            );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
