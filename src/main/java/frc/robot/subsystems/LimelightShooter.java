// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `LimelightShooter` class represents a subsystem that interacts with the Limelight camera for shooting purposes.
 * It provides methods to control the camera mode, lights, pipeline, and retrieve information about detected objects.
 */
public class LimelightShooter extends SubsystemBase {
  public final String LIMELIGHT = "limelight-shooter";
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
    PoseEstimation(0);

    public int pipelineNum;

    private Pipeline(int pipelineNum){
      this.pipelineNum = pipelineNum;
    }
  }

  /**
   * Limelight Subsystem
   */
  public LimelightShooter() {
    turnOnLimelight();
  }

  /**
   * Switches the camera to active and turns on the lights
   */
  public void turnOnLimelight() {
    table.getEntry("camMode").setNumber(0);
    setLights(LightMode.DEFAULT);
  }

  /**
   * Switches the camera to inactive and turns off the lights
   */
  public void turnOffLimelight() {
    table.getEntry("camMode").setNumber(1);
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
    table.getEntry("ledMode").setNumber(lightMode.lightNum);
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
    table.getEntry("pipeline").setNumber(pipeline.pipelineNum);
  }

  /**
   * Scan Apriltag if you're in the right pipeline
   * @return Id of Apriltag
   */
  public Double getApriltagID() {
    double id = table.getEntry("tid").getDouble(0);
    return (id != 0) ? id : Double.NaN;
    // return 1;
  }

  /**
   * Finds the pose of the robot when detecting 3D april tags
   * @return An array of doubles in the order of X, Y, Z, Roll, Pitch, Yaw
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

  /**
   * Calculates the distance in the X-axis between the robot's pose and the target speaker.
   * 
   * @return The distance in the X-axis between the robot's pose and the target speaker. If the robot's pose is not available, returns Double.NaN.
   */
  public Double getXDistance() {
    double[] pose = getRobotPose();

    if (pose[0] == 0 && pose[1] == 0) {
      return Double.NaN;
    }

    var alliance = DriverStation.getAlliance();
    double xDistance = Double.NaN;
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
    } else if (alliance.get().equals(Alliance.Red)) {
      xDistance = Math.abs(redSpeakerX - pose[0]);
    } else if (alliance.get().equals(Alliance.Blue)) {
      xDistance = Math.abs(blueSpeakerX - pose[0]);
    }
    return xDistance;
  }

  /**
   * Calculates the distance in the Y-axis from the robot's current position to the target speaker.
   * 
   * @return The distance in the Y-axis from the robot's current position to the target speaker. If the robot's pose is not available, returns Double.NaN.
   */
  public Double getYDistance() {
    double[] pose = getRobotPose();

    if (pose[0] == 0 && pose[1] == 0) {
      return Double.NaN;
    }

    var alliance = DriverStation.getAlliance();
    double yDistance = Double.NaN;
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
    } else if (alliance.get().equals(Alliance.Red)) {
      yDistance = Math.abs(redSpeakerY - pose[1]);
    } else if (alliance.get().equals(Alliance.Blue)) {
      yDistance = Math.abs(blueSpeakerY - pose[1]);
    }
    return yDistance;
  }

  /**
   * Calculates the distance from the goal using the x and y distances.
   * 
   * @return The distance from the goal.
   */
  public Double getDistanceFromGoal() {
    Double xDistance = getXDistance();
    Double yDistance = getYDistance();

    return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
  }

  /**
   * Calculates the angle from the robot to the goal using the x and y distances.
   * 
   * @return The angle from the robot to the goal in radians.
   */
  public Double getAngleFromGoal() {
    Double xDistance = getXDistance();
    Double yDistance = getYDistance();

    return Math.atan2(xDistance, yDistance);
  }

  /**
   * @return X position of the object (degrees)
   */
  public Double getTX() {
    double tX = table.getEntry("tx").getDouble(0);
    return (tX != 0) ? tX : Double.NaN;
    // return 1;
  }

  /**
   * @return Y position of the object (degrees)
   */
  public Double getTY() {
    double tY = table.getEntry("ty").getDouble(0);
    return (tY != 0) ? tY : Double.NaN;
    // return 1;
  }

  /**
   * @return Area of the screen the object takes up
   */
  public Double getTA() {
    double tA = table.getEntry("ta").getDouble(0);
    return (tA != 0) ? tA : Double.NaN;
    // return 1;
  }

  /**
   * @return Skew (rotation) of the object
   */
  public Double getTS() {
    double tS = table.getEntry("ts").getDouble(0);
    return (tS != 0) ? tS : Double.NaN;
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
    // System.out.println("TY: " + getTY());
    // System.out.println("start");
    // for (double d : getRobotPose()) {
    //   System.out.println(d);
    // }
    // System.out.println("end");

    // SmartDashboard.putNumber("tX", getTX());
    // SmartDashboard.putNumber("tY", getTY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
