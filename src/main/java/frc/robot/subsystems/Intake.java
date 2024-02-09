// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Utilities;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(13);

  private DigitalInput noteSensor = new DigitalInput(1);

  VelocityVoltage velocityControlFeed;
  VoltageOut voltageControl;
  NeutralOut stopMode;
  
  /**
   * Creates a new Intake.
   */
  public Intake() {
    initIntakeMotor();

    velocityControlFeed = new VelocityVoltage(0, 
                                          0, 
                                          true, 
                                          0, 
                                          1, 
                                          false, 
                                          false, 
                                          false
                                          );

    voltageControl = new VoltageOut(0, 
                                    false, 
                                    false, 
                                    false, 
                                    false);

    stopMode = new NeutralOut();
  }
   
  /**
   * Initialize the intake motor
   */
  public void initIntakeMotor() {
    System.out.println("Intake Safety: " + intakeMotor.isSafetyEnabled());

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
      status = intakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @param timeout in seconds; use 0 for no timeout
   * @return a command that will run the intake motor
   */
  public Command feedCommand(double velocity, double acceleration, double timeout){
    return new Command() {
      private long startingTime;

      @Override
      public void initialize() {
        addRequirements(Intake.this);
        startingTime = System.currentTimeMillis();
      }

      @Override
      public void execute() {
        feedMotor(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopIntakeMotor();
      }

      @Override
      public boolean isFinished() {
        System.out.println((System.currentTimeMillis() - startingTime));
        if((timeout != 0.0) && (System.currentTimeMillis() - startingTime) >= (timeout * 1000)) {
          return true;
        } else {
          return false;
        }
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void feedMotor(double velocity, double acceleration) {
    intakeMotor.setControl(velocityControlFeed
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );
  }

  /**
   * Run the intake motor at a given voltage
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runVoltageCommand(double voltage){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
      }

      @Override
      public void execute() {
        runVoltage(voltage);
      }

      // @Override
      // public void end(boolean interrupted) {
      //   stopIntakeMotor();
      // }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the intake motor at a given voltage
   * @param voltage in volts
   */
  public void runVoltage(double voltage) {
    intakeMotor.setControl(voltageControl
                            .withOutput(voltage)
                          );
  }


  public Command runIntakePercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Intake.this);
      }

      @Override
      public void execute() {
        intakeMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        intakeMotor.set(0);
      }
    };
  }

  
  public void stopIntakeMotor() {
    intakeMotor.setControl(stopMode);
  }

  public Command waitUntilTripped() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return getNoteSensor();
      }
    };
  }

  public boolean getNoteSensor() {
    return noteSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pdp.getCurrent(16));
    // System.out.println(getDistanceSensorTripped());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
