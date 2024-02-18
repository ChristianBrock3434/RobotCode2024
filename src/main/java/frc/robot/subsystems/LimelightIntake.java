// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightIntake extends SubsystemBase {
  public final String LIMELIGHT = "limelight-intake";
  NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);

  public static enum LightMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    public int lightNum;

    private LightMode(int lightNum){
      this.lightNum = lightNum;
    }
  };

  public static enum Pipeline {
    Note(0);
    // AprilTag3D(1);

    public int pipelineNum;

    private Pipeline(int pipelineNum){
      this.pipelineNum = pipelineNum;
    }
  }

  /**
   * Limelight Subsystem
   */
  public LimelightIntake() {
    turnOnLimelight();
  }

  /**
   * Switches the camera to active and turns on the lights
   */
  public void turnOnLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(0);
    setLights(LightMode.DEFAULT);
  }

  /**
   * Switches the camera to inactive and turns off the lights
   */
  public void turnOffLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(1);
    setLights(LightMode.OFF);
  }

  /**
   * Set the Light mode of the limelight
   * @param lightMode LightMode enum value
   * @return Command to be scheduled
   */
  public Command setLightsCommand(LightMode lightMode) {
    return new Command() {
        @Override
        public void initialize() {
            setLights(lightMode);
        }
    };
  }

  /**
   * Sets light mode of the limelight
   * @param lightMode LightMode enum value
   */
  public void setLights(LightMode lightMode) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ledMode").setNumber(lightMode.lightNum);
  } 

  /**
   * Switches what the limelight is detecting
   * @param pipeline Pipeline enum value
   * @return Command to be scheduled
   */
  public Command setLimelightPipelineCommand(Pipeline pipeline) {
    return new Command() {
        @Override
        public void initialize() {
            setLimelightPipeline(pipeline);
        }
    };
  }

  /**
   * Switches what the limelight is detecting
   * @param pipeline Pipeline enum value
   */
  public void setLimelightPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("pipeline").setNumber(pipeline.pipelineNum);
  }

  public Command prepareForNote() {
    return new Command() {
      @Override
      public void initialize() {
        turnOnLimelight();
        setLimelightPipeline(LimelightIntake.Pipeline.Note);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Scan Apriltag if you're in the right pipeline
   * @return Id of Apriltag
   */
  public double getApriltagID() {
    double id = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tid").getDouble(0);
    // SmartDashboard.putNumber("Apriltag id", id);
    return id;
    // return 1;
  }

  /**
   * Prints the name of piece that the Limelight detects
   * @param pipeline Pipeline enum value of the piece to detect
   * @return Command to be scheduled
   */
  public Command printPieceNameCommand(Pipeline pipeline) {
    return new Command() {
        @Override
        public void initialize() {
            turnOnLimelight();
            setLimelightPipeline(pipeline);
        }

        @Override
        public void execute() {
            printPieceName();
        }

        @Override
        public void end(boolean interrupted) {
            turnOffLimelight();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    };
  }

  /**
   * Prints the name of piece that the Limelight detects
   */
  public void printPieceName() {
    System.out.println(NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tclass").getString(null));
  }

  /**
   * Finds the pose of the robot when detecting 3D april tags
   * @return An array of doubles in the order of X, Y, Z, Pitch, Yaw, Roll
   */
  public double[] getRobotPose() {
    double[] pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    try{
      pose = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("botpose").getDoubleArray(pose);
      // SmartDashboard.putNumber("PoseX", pose[0]);
      // SmartDashboard.putNumber("PoseY", pose[1]);
      // SmartDashboard.putNumber("PoseZ", pose[2]);
      // SmartDashboard.putNumber("PosePitch", pose[3]);
      // SmartDashboard.putNumber("PoseYaw", pose[4]);
      // SmartDashboard.putNumber("PoseRoll", pose[5]);
    } catch(ArrayIndexOutOfBoundsException e) {
      // System.out.println("No 3D April tag detected");
    }
    return pose;
    // return null;
  }

  public double getDistanceFromGoal() {
    double[] pose = getRobotPose();

    if (pose[0] == 0 && pose[1] == 0) {
      return Double.NaN;
    }

    //TODO: add support for red alliance
    double xDistance = Math.abs(blueSpeakerX - pose[0]);
    double yDistance = Math.abs(blueSpeakerY - pose[1]);

    return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
  }

  /**
   * @return X position of the object (degrees)
   */
  public double getTX() {
    return table.getEntry("tx").getDouble(Double.NaN);
    // return 1;
  }

  /**
   * @return Y position of the object (degrees)
   */
  public double getTY() {
    return table.getEntry("ty").getDouble(Double.NaN);
    // return 1;
  }

  /**
   * @return Area of the screen the object takes up
   */
  public double getTA() {
    return table.getEntry("ta").getDouble(Double.NaN);
    // return 1;
  }

  /**
   * @return Skew (rotation) of the object
   */
  public double getTS() {
    return table.getEntry("ts").getDouble(Double.NaN);
    // return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double[] pose = getRobotPose();
    // System.out.println("X: " + pose[0]);
    // System.out.println("Y: " + pose[1]);
    // System.out.println("Distance: " + getDistanceFromGoal());
    // System.out.println("TX: " + getTX());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
