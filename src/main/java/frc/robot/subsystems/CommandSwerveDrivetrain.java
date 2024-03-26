package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private double offset = 0;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        setBrakeMode();
        // this.getPigeon2().reset();
        // applyCurrentLimiting();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        setBrakeMode();
        // this.getPigeon2().reset();
        // applyCurrentLimiting();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Apply current limiting to the drive and steer motors
     */
    public void applyCurrentLimiting() {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 45;
        for (var module : this.Modules) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = module.getDriveMotor().getConfigurator().apply(configs);
                if (status.isOK()) break;
            }
            if(!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }
        }

        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 25;
        for (var module : this.Modules) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = module.getSteerMotor().getConfigurator().apply(configs);
                if (status.isOK()) break;
            }
            if(!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }
        }
    }

    /**
     * Remove current limiting from the drive and steer motors
     */
    public void removeCurrentLimiting() {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimitEnable = false;
        for (var module : this.Modules) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = module.getDriveMotor().getConfigurator().apply(configs);
                if (status.isOK()) break;
            }
            if(!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }

            status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = module.getSteerMotor().getConfigurator().apply(configs);
                if (status.isOK()) break;
            }
            if(!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }
        }
    }

    /**
     * Configure the path planner for the swerve drivetrain
     */
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::resetOrientation,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            this::shouldFlipPath, 
            this); // Subsystem for requirements
    }

    public void setBrakeMode() {
        configNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        configNeutralMode(NeutralModeValue.Coast);
    }

    public void setPose(Pose2d newPose) {
        Pose2d pose = new Pose2d(new Translation2d(newPose.getX(), newPose.getY()), this.getPose().getRotation());
        // this.m_odometry.resetPosition(this.getRotation(), m_modulePositions, pose);
        seedFieldRelative(pose);
    }

    public void setPose(Pose2d newPose, double timestampSeconds) {
        Pose2d pose = new Pose2d(new Translation2d(newPose.getX(), newPose.getY()), this.getPose().getRotation());
        this.m_odometry.addVisionMeasurement(pose, timestampSeconds);
    }

    /**
     * Sets the rotation of the robot to zero
     */
    public void resetOrientation() {
        seedFieldRelative();
        offset = m_pigeon2.getAngle();
    }

    /**
     * Sets the rotation of the robot to the given location
     * @param location the location to set the rotation to
     */
    public void resetOrientation(Pose2d location) {
        seedFieldRelative(location);
        offset = m_pigeon2.getAngle() + location.getRotation().getDegrees();
    }

    private boolean shouldFlipPath() {
        return false;
    }

    /**
     * Apply a swerve request to the drivetrain
     * @param requestSupplier the supplier of the swerve request
     * @return a command that will apply the swerve request
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Get an auto based off of the name
     * @param pathName the name of the path
     * @return a command that will run the path
     */
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * Get the end state of a path
     * @param autoName the name of the auto to grab from
     * @param index the index of the path in that auto
     * @return the end state of the path
     */
    public State getEndPath(String autoName, int index){
        var pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        var path = pathGroup.get(index);

        ChassisSpeeds startingChassisSpeed = new ChassisSpeeds(0, 0, 0);
        Rotation2d rot;

        try{
            var previousPath = pathGroup.get(index-1);
            var startingState = previousPath.getGoalEndState();
            rot = startingState.getRotation();
        } catch (IndexOutOfBoundsException e){
            rot = getRotation();
        }

        var trajectory = new PathPlannerTrajectory(path, startingChassisSpeed, rot);
        return trajectory.getEndState();
    }

    /**
     * wait until the drivetrain is not moving
     * @return a command that will wait until the drivetrain is not moving
     */
    public Command waitUntilNotMoving() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return !isMoving();
            }
        };
    }

    /**
     * Checks if the drivetrain is moving
     * @return if the drivetrain is moving
     */
    public boolean isMoving() {
        double flSpeed = this.getModule(0).getDriveMotor().getVelocity().getValueAsDouble();
        double frSpeed = this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble();
        double blSpeed = this.getModule(2).getDriveMotor().getVelocity().getValueAsDouble();
        double brSpeed = this.getModule(3).getDriveMotor().getVelocity().getValueAsDouble();
        return (flSpeed > 0.1) || (frSpeed > 0.1) || (blSpeed > 0.1) || (brSpeed > 0.1);
    }

    /**
     * print the current acceleration of the robot
     */
    public void printAcceleration() {
        double xAccel = m_pigeon2.getAccelerationX().getValueAsDouble();
        double yAccel = m_pigeon2.getAccelerationY().getValueAsDouble();
        double totalAccel = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2));
        // System.out.println("x Acceleration: " + xAccel);
        // System.out.println("y Acceleration: " + yAccel);
        System.out.println("total Acceleration: " + totalAccel);
    }

    /**
     * check the acceleration of the robot
     * @param targetAccel the target acceleration
     * @return the difference between the target and actual acceleration
     */
    public double checkAcceleration(double targetAccel) {
        double xAccel = m_pigeon2.getAccelerationX().getValueAsDouble();
        double yAccel = m_pigeon2.getAccelerationY().getValueAsDouble();
        double totalAccel = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2));
        // System.out.println("x Acceleration: " + xAccel);
        // System.out.println("y Acceleration: " + yAccel);
        return totalAccel - targetAccel;
    }

    /**
     * get the current rotation of the robot
     * @return the current rotation of the robot
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(-MathUtil.inputModulus(m_pigeon2.getAngle() - offset, -180, 180));
    }

    /**
     * get the current rotation of the robot in degrees
     * @return the current rotation of the robot in degrees
     */
    public double getDegrees() {
        return -MathUtil.inputModulus(m_pigeon2.getAngle() - offset, -180, 180);
    }

    /**
     * get the current position of the robot
     * @return the current position of the robot
     */
    public Pose2d getPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    /**
     * get the offset applied to a given swerve module
     * @param index the index of the swerve module
     * @return the offset applied to the swerve module
     */
    public double getOffset(int index) {
        return this.getModule(index).getCANcoder().getPosition().getValueAsDouble();
    }

    /**
     * get the current ChassisSpeeds of the robot
     * @return the current ChassisSpeeds of the robot
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * get the current x position of the robot
     * @return the current x position of the robot
     */
    public double getX() {
        return getPose().getX();
    }

    /**
     * get the current y position of the robot
     * @return the current y position of the robot
     */
    public double getY() {
        return getPose().getY();
    }

    /**
   * Calculates the distance in the X-axis between the robot's pose and the target speaker.
   * 
   * @return The distance in the X-axis between the robot's pose and the target speaker. If the robot's pose is not available, returns Double.NaN.
   */
  public Double getXDistance() {
    double xPose = getPose().getX();

    var alliance = DriverStation.getAlliance();
    double xDistance = Double.NaN;
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
    } else if (alliance.get().equals(Alliance.Red)) {
      xDistance = Math.abs(redSpeakerX - xPose);
    } else if (alliance.get().equals(Alliance.Blue)) {
      xDistance = Math.abs(blueSpeakerX - xPose);
    }
    return xDistance;
  }

  /**
   * Calculates the distance in the Y-axis from the robot's current position to the target speaker.
   * 
   * @return The distance in the Y-axis from the robot's current position to the target speaker. If the robot's pose is not available, returns Double.NaN.
   */
  public Double getYDistance() {
    double yPose = getPose().getY();

    var alliance = DriverStation.getAlliance();
    double yDistance = Double.NaN;
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("ALLIANCE IS EMPTY, SELECT AN ALLIANCE", true);
    } else if (alliance.get().equals(Alliance.Red)) {
      yDistance = Math.abs(redSpeakerY - yPose);
    } else if (alliance.get().equals(Alliance.Blue)) {
      yDistance = Math.abs(blueSpeakerY - yPose);
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

    @Deprecated
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        // System.out.println(getRotation().getDegrees());
        // SmartDashboard.putNumber("Rotation", getRotation().getDegrees());
    }
}
