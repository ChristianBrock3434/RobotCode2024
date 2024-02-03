// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static final double[][] distanceMap = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  };

  private TalonFX leftShooterMotor = new TalonFX(15);
  private TalonFX rightShooterMotor = new TalonFX(16);

  VelocityVoltage velocityControl;
  VoltageOut sitControl;
  NeutralOut stopMode;


  private static final double leftSideMultiplier = 1.00;
  private static final double rightSideMultiplier = 1.00;
  
  /**
   * Creates a new Intake.
   */
  public Shooter() {
    initMotors();

    velocityControl = new VelocityVoltage(0, 
                                          0, 
                                          true, 
                                          0.2,
                                          0, 
                                          false, 
                                          false, 
                                          false
                                          );

    sitControl = new VoltageOut(0.5,
                                false, 
                                false, 
                                false, 
                                false);

    stopMode = new NeutralOut();
  }

  /**
   * Initialize the both shooter motors
   */
  public void initMotors() {
    System.out.println("Left Shooter Safety: " + leftShooterMotor.isSafetyEnabled());
    System.out.println("Right Shooter Safety: " + rightShooterMotor.isSafetyEnabled());

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.5; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 1; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightShooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftShooterMotor.getConfigurator().apply(configs);
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
   * @return a command that will run the intake motor
   */
  public Command runShooterCommand(double velocity, double acceleration){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        runShooter(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        stopShooter();
      }
    };
  }

  /**
   * Run the shooter at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the shooter
   */
  public Command speedUpShooter(double velocity, double acceleration){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
        runShooter(velocity, acceleration);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * @param velocity in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runShooter(double velocity, double acceleration) {
    leftShooterMotor.setControl(velocityControl
                            .withVelocity(velocity * leftSideMultiplier)
                            .withAcceleration(acceleration)
                          );

    rightShooterMotor.setControl(velocityControl
                            .withVelocity(velocity * rightSideMultiplier)
                            .withAcceleration(acceleration)
                          );
  }


  public Command runShooterPercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        leftShooterMotor.set(speed * leftSideMultiplier);
        rightShooterMotor.set(speed * rightSideMultiplier);
      }

      @Override
      public void end(boolean interrupted) {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
      }
    };
  }

  
  public void stopShooter() {
    // leftShooterMotor.setControl(stopMode);
    // rightShooterMotor.setControl(stopMode);
    leftShooterMotor.setControl(sitControl);
    rightShooterMotor.setControl(sitControl);
  }

  public Command checkIfAtSpeed(double velocity) {
    return new Command() {
      @Override
      public void initialize() {
      }

      @Override
      public void execute() {
      }

      @Override
      public void end(boolean interrupted) {

      }

      @Override
      public boolean isFinished() {
        boolean leftShooterSpeed = leftShooterMotor.getVelocity().getValueAsDouble() >= velocity * leftSideMultiplier;
        boolean rightShooterSpeed = rightShooterMotor.getVelocity().getValueAsDouble() >= velocity * rightSideMultiplier;
        return rightShooterSpeed;
      }
    };
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pdp.getCurrent(16));
    // System.out.println("Right Velocity:" + rightShooterMotor.getVelocity().getValueAsDouble());
    // System.out.println("Left Velocity:" + leftShooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
