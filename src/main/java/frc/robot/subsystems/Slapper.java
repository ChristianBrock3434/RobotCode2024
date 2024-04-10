package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.SlapperConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Slapper subsystem controls the slapper motor, which is used to keep the note in the amp
 */
public class Slapper extends SubsystemBase {
  private TalonFX slapperMotor = new TalonFX(20);

  MotionMagicVoltage motionMagicControl;
  NeutralOut stopMode;

  public Slapper() {
    initSlapperMotor();

    motionMagicControl = new MotionMagicVoltage(0, 
                                                true, 
                                                -0.65,
                                                0,
                                                false,
                                                false,
                                                false
                                              );

    stopMode = new NeutralOut();
  }

  public void initSlapperMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    slapperMotor.setPosition(0);

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 25;

    configs.MotionMagic.MotionMagicCruiseVelocity = 10;
    configs.MotionMagic.MotionMagicAcceleration = 35;
    configs.MotionMagic.MotionMagicJerk = 40;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 25; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.3; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = slapperMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }

  /**
   * Run the Slapper motor to a given position
   * @param position in degrees
   * @return a command that will run the Slapper motor
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
   * Creates a command that sets the position of the slapper based on the given position supplier.
   * 
   * @param position the supplier that provides the desired position of the slapper
   * @return the command that sets the position of the slapper
   */
  public Command setPositionCommand(DoubleSupplier position) {
    return new Command() {
      @Override
      public void execute() {
        setPosition(position.getAsDouble());
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Run the Slapper motor to a given position
   * @param position in degrees
   */
  public void setPosition(double position) {
    slapperMotor.setControl(motionMagicControl
                              .withPosition(position  * slapperTicksPerDegree)
                            );
  }

  /**
   * Stops the Slapper motor.
   * 
   * @return the command to stop the Slapper motor
   */
  public Command stopMotor() {
    return new Command() {
      @Override
      public void execute() {
        stop();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Stops the Slapper motor.
   */
  public void stop() {
    slapperMotor.setControl(stopMode);
  }
    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Slapper pos", slapperMotor.getPosition().getValueAsDouble() / slapperTicksPerDegree);
  }
}
