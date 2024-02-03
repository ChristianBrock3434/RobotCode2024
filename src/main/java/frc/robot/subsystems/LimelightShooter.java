// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightShooter extends SubsystemBase {
  public final String LIMELIGHT = "limelight-shooter";
  private double tx = 0;
  private double ty = 0;
  private double ta = 0;
  private double ts = 0;

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
    AprilTag3D(0);

    public int pipelineNum;

    private Pipeline(int pipelineNum){
      this.pipelineNum = pipelineNum;
    }
  }

  /**
   * Limelight Subsystem
   */
  public LimelightShooter() {
    updateLimeLight();

    // turnOffLimelight();
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

  /**
   * Gets the position of what the Limelight is detecting
   * @return nothing, we store these values
   */
  public void updateLimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    ts = table.getEntry("ts").getDouble(0);

    // table.putValue("ledMode", 3);

    // final ShuffleboardTab tab = Shuffleboard.getTab(LIMELIGHT);
    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("ts", ts);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
  }

  /**
   * Resets limelight and prepares for the next detection
   */
  public void resetLimelight() {
    tx = 0;
    ty = 0;
    ta = 0;
    ts = 0;

    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("ts", ts);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
    // SmartDashboard.putNumber("Apriltag id", -1);

    turnOffLimelight();
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

  public Double getDistanceFromGoal() {
    double[] pose = getRobotPose();

    if (pose[0] == 0 && pose[1] == 0) {
      return null;
    }

    double xDistance = Math.abs(blueSpeakerX - pose[0]);
    double yDistance = Math.abs(blueSpeakerY - pose[1]);

    return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
  }

  /**
   * @return X position of the object (degrees)
   */
  public double getTX() {
    return tx;
    // return 1;
  }

  /**
   * @return Y position of the object (degrees)
   */
  public double getTY() {
    return ty;
    // return 1;
  }

  /**
   * @return Area of the screen the object takes up
   */
  public double getTA() {
    return ta;
    // return 1;
  }

  /**
   * @return Skew (rotation) of the object
   */
  public double getTS() {
    return ts;
    // return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double[] pose = getRobotPose();
    // System.out.println("X: " + pose[0]);
    // System.out.println("Y: " + pose[1]);
    // System.out.println("Distance: " + getDistanceFromGoal());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
