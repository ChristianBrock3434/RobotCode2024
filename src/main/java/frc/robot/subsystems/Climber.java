package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
  private TalonFX climberMotor = new TalonFX(18);

  private VoltageOut voltageControl;
  private NeutralOut StopMode;

  public Climber() {
    initClimberMotor();

    voltageControl = new VoltageOut(0.0, 
                                    true, 
                                    false, 
                                    false, 
                                    false
                                    );
                              
    StopMode = new NeutralOut();
  }

  public void initClimberMotor() {
    resetClimber();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = climberMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public Command climberPercentControl(double percent) {
    return new Command() {
      @Override
      public void execute() {
        climberMotor.set(percent);
      }

      @Override
      public void end(boolean interrupted) {
        climberMotor.set(0);
      }
    };
  }

  /**
   * Run the Climber motor at a given voltage, Stopping at it's max height and lowest height
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runLimitedVoltageCommand(double voltage){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Climber.this);
      }

      @Override
      public void execute() {
        if (climberMotor.getPosition().getValueAsDouble() < minClimberHeight && voltage < 0) {
          stopClimberMotor();
        } else if (climberMotor.getPosition().getValueAsDouble() > maxClimberHeight && voltage > 0) {
          stopClimberMotor();
        } else {
          runVoltage(voltage);
        }
      }

      @Override
      public void end(boolean interrupted) {
        stopClimberMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Run the Climber motor at a given voltage
   * @param voltage in volts
   * @return a command that will run the intake motor
   */
  public Command runVoltageCommand(double voltage){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Climber.this);
      }

      @Override
      public void execute() {
        runVoltage(voltage);
      }

      @Override
      public void end(boolean interrupted) {
        stopClimberMotor();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Run the Climber motor at a given voltage
   * @param voltage in volts
   */
  public void runVoltage(double voltage) {
    climberMotor.setControl(voltageControl
                            .withOutput(voltage)
                          );
  }

  /**
   * Stop the Climber motor
   */
  public void stopClimberMotor() {
    climberMotor.setControl(StopMode);
  }

  /**
   * Reset the Climber encoder
   */
  public void resetClimber() {
    climberMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Climber Position: " + climberMotor.getPosition().getValueAsDouble());
  }
}
