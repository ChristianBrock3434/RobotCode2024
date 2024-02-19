// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
  // TODO: Tune the table
  // Distance, Angle, Speed
  private static final Double[][] distanceMap = {
    {1.4, 1.0, 65.0},
    {1.6, 3.0, 65.0},
    {1.8, 6.0, 65.0},
    {1.9, 8.0, 65.0},
    {2.05, 10.0, 65.0},
    {2.25, 12.0, 65.0},
    {2.45, 15.0, 65.0},
    {2.6, 17.0, 65.0},
    {2.75, 19.0, 65.0},
    {2.9, 20.5, 65.0},
    {3.0, 21.5, 65.0},
    {3.2, 22.5, 65.0},
    {3.35, 22.5, 65.0},
    {3.5, 23.0, 65.0},
    {3.6, 24.0, 65.0},
    {3.7, 25.0, 65.0},
    {3.85, 27.25, 65.0},
    {4.03, 27.5, 65.0},
    {4.2, 28.25, 65.0},
    {4.35, 29.25, 65.0},
    {4.5, 29.5, 65.0},
    {4.65, 29.75, 65.0}, //come back
    {4.8, 31.5, 65.0},
    {5.0, 32.5, 65.0}
  };

  private TalonFX leftShooterMotor = new TalonFX(15);
  private TalonFX rightShooterMotor = new TalonFX(16);

  VelocityVoltage velocityControl;
  VoltageOut sitControl;
  NeutralOut stopMode;
  
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
                                          true
                                          );

    sitControl = new VoltageOut(2,
                                false, 
                                false, 
                                false, 
                                true);

    stopMode = new NeutralOut();

    stopShooter();
  }

  /**
   * Initialize the both shooter motors
   */
  public void initMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.5; //0.5 // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 1.0; //1.0 // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightShooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

  public Command speedUpShooterSupplier(DoubleSupplier velocity, double acceleration){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
        if (((Double) velocity.getAsDouble()).equals(Double.NaN)) {
          stopShooter();
        } else {
          runShooter(velocity.getAsDouble(), acceleration);
        }
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
                            .withVelocity(velocity)
                            .withAcceleration(acceleration)
                          );

    rightShooterMotor.setControl(velocityControl
                            .withVelocity(velocity)
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
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
      }
    };
  }

  
  public void stopShooter() {
    leftShooterMotor.setControl(stopMode);
    rightShooterMotor.setControl(stopMode);
    // leftShooterMotor.setControl(sitControl);
    // rightShooterMotor.setControl(sitControl);
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
        boolean leftShooterSpeed = leftShooterMotor.getVelocity().getValueAsDouble() >= velocity;
        boolean rightShooterSpeed = rightShooterMotor.getVelocity().getValueAsDouble() >= velocity;
        return leftShooterSpeed || rightShooterSpeed;
      }
    };
  }

  public Command checkIfAtSpeedSupplier(DoubleSupplier velocity) {
    return new Command() {
      @Override
      public void initialize() {
      }

      @Override
      public void execute() {
        // System.out.println("Speed required: " + velocity.getAsDouble());
      }

      @Override
      public void end(boolean interrupted) {

      }

      @Override
      public boolean isFinished() {
        if (((Double) velocity.getAsDouble()).equals(Double.NaN)) {
          return true;
        }
        boolean leftShooterSpeed = leftShooterMotor.getVelocity().getValueAsDouble() >= velocity.getAsDouble();
        boolean rightShooterSpeed = rightShooterMotor.getVelocity().getValueAsDouble() >= velocity.getAsDouble();
        return leftShooterSpeed || rightShooterSpeed;
      }
    };
  }
  
  
  public Double[] getAngleAndSpeed(Double distance) {
    Double[] emptyVal = {Double.NaN, Double.NaN, Double.NaN};
    if (distance.equals(Double.NaN)) return emptyVal;

    for (int i = 0; i < distanceMap.length; i++) {
      double curDis = distanceMap[i][0];

      if (distance > curDis) {
        continue;
      }

      try {
        double prevDis = distanceMap[i-1][0];
        prevDis = Math.abs(distance - prevDis);
        curDis = Math.abs(curDis - distance);
        
        double totalDis = prevDis + curDis;
        curDis = curDis / totalDis;
        prevDis = prevDis / totalDis;

        Double[] arr = {distance,
                        distanceMap[i][1] * curDis + distanceMap[i-1][1] * prevDis,
                        distanceMap[i][2] * curDis + distanceMap[i-1][2] * prevDis
                      };

        // return (curDis < prevDis) ? distanceMap[i] : distanceMap[i-1];
        return arr;
      } 
      catch (ArrayIndexOutOfBoundsException e) {
        // for (double j : distanceMap[i]) {
        //   System.out.println(j);
        // }
        return distanceMap[i];
      }
    }
    if (distance < distanceMap[distanceMap.length-1][0] + 0.1) {
      return distanceMap[distanceMap.length-1];
    }
    return emptyVal;
  }

  public double[] getAngleAndSpeedEquation(Double distance) {
    double[] emptyVal = {Double.NaN, Double.NaN, Double.NaN};
    if (distance < 0 || distance > 5.1) return emptyVal;

    double angle = -14.6 + 29.1 * Math.log(distance);
    if (angle < 0) {
      angle = 0;
    }

    // distance = 0.03953629709 * angle + 0.0733530393 * speed + 0.05272464358
    // (distance - 0.05272464358 - 0.03953629709 * angle) / 0.0733530393 = speed
    // double speed = (distance - 0.05272464358 - 0.03953629709 * angle) / 0.0733530393;
    double speed = 41.9 + -3.97 * distance + 1.74 * Math.pow(distance, 2);
    if (distance <= 2.9) {
      speed = 40;
    }

    double[] arr = {distance, angle, speed};
    return arr;
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
