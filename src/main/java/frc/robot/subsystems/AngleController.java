package frc.robot.subsystems;

import static frc.robot.Constants.AngleControllerConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleControllerConstants;

public class AngleController extends SubsystemBase{
  private TalonFX angleMotor = new TalonFX(19);

  MotionMagicVoltage motionMagicControl;
  NeutralOut stopMode;

  public AngleController() {
    initAngleMotor();

    motionMagicControl = new MotionMagicVoltage(0, 
                                                true, 
                                                0.0,
                                                0,
                                                false,
                                                false,
                                                false
                                              );
              
    stopMode = new NeutralOut();
  }

  public void initAngleMotor() {
    angleMotor.setPosition(angleStartingPosition);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    configs.MotionMagic.MotionMagicCruiseVelocity = 15;
    configs.MotionMagic.MotionMagicAcceleration = 20;
    configs.MotionMagic.MotionMagicJerk = 50;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 25; //50 // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; //4 // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Run the Angle Controller motor to a given position
   * @param position in rotations
   * @return a command that will run the Angle Controller motor
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
   * Supplier because the codebase hates you
   * @param position
   * @return
   */
  public Command setPositionCommandSupplier(DoubleSupplier position) {
    return new Command() {
      @Override
      public void execute() {
        if (position.getAsDouble() != Double.NaN) {
          setPosition(position.getAsDouble());
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the Angle Controller motor to a given position
   * @param position in rotations
   */
  public void setPosition(double position) {
    angleMotor.setControl(motionMagicControl
                              .withPosition(position)
                            );
  }

  public Command anglePercentControl(double power) {
    return new Command() {
      @Override
      public void execute() {
        angleMotor.set(power);
      }

      @Override
      public void end(boolean interrupted) {
        angleMotor.set(0);
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
        double currentPosition = angleMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - setPosition) <= 0.1;
      }
    };
  }

  public Command waitUntilAtPositionSupplier(DoubleSupplier setPosition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        if (setPosition.getAsDouble() == Double.NaN) {
          return true;
        }
        double currentPosition = angleMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - setPosition.getAsDouble()) <= 0.05;
      }
    };
  }

  public void resetEncoder() {
    angleMotor.setPosition(angleStartingPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(angleMotor.getPosition().getValueAsDouble() / AngleControllerConstants.angleTicksPerDegree);
  }
}
