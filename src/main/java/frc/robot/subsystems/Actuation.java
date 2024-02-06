// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ActuationConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuation extends SubsystemBase {
  private TalonFX actuationMotor = new TalonFX(14);

  private DigitalInput actuationLimitSwitch = new DigitalInput(0);

  MotionMagicVoltage motionMagicControl;
  NeutralOut stopMode;
  
  /**
   * Creates a new Intake.
   */
  public Actuation() {
    initActuationMotor();

    motionMagicControl = new MotionMagicVoltage(0, 
                                                true, 
                                                1,
                                                0,
                                                false,
                                                false,
                                                false
                                              );

    stopMode = new NeutralOut();
  }
  

  /**
   * Initialize the actuation motor
   */
  public void initActuationMotor() {
    // actuationMotor.setNeutralMode(NeutralModeValue.Brake);

    actuationMotor.setPosition(actuationStartPosition);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    configs.MotionMagic.MotionMagicCruiseVelocity = 15;
    configs.MotionMagic.MotionMagicAcceleration = 20;
    configs.MotionMagic.MotionMagicJerk = 50;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 40; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
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

      @Override
      public boolean isFinished() {
        return true;
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


 

  public Command runActuatorPercent(double speed) {
    return new Command() {
      @Override
      public void execute() {
        actuationMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        actuationMotor.set(0);
      }
    };
  }

  public Command resetEncoderCommand() {
    return new Command() {
      @Override
      public void execute() {
        resetEncoder();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command waitUntilAtPosition(double setPosition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        double currentPosition = actuationMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - setPosition) <= 0.3;
      }
    };
  }

  public void resetEncoder() {
    actuationMotor.setPosition(actuationStartPosition);
  }


  public Boolean getLimitSwitch() {
    return !actuationLimitSwitch.get();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(getLimitSwitch());
    // System.out.println(actuationMotor.getPosition().getValueAsDouble());
    // System.out.println(actuationMotor.getMotorVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
