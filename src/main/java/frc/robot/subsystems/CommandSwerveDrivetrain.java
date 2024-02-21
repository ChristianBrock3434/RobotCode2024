package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        applyCurrentLimiting();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        applyCurrentLimiting();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    //TODO: Find out why this breaks the drivetrain
    private void applyCurrentLimiting() {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 50;
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

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
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

    private boolean shouldFlipPath() {
        return false;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public State getEndPath(String autoName, int index){
        var pathGroup = new PathPlannerAuto(autoName).getPathGroupFromAutoFile(autoName);
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

    public Command waitUntilNotMoving() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return !isMoving();
            }
        };
    }

    public boolean isMoving() {
        double flSpeed = this.getModule(0).getDriveMotor().getVelocity().getValueAsDouble();
        double frSpeed = this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble();
        double blSpeed = this.getModule(2).getDriveMotor().getVelocity().getValueAsDouble();
        double brSpeed = this.getModule(3).getDriveMotor().getVelocity().getValueAsDouble();
        return (flSpeed > 3) || (frSpeed > 3) || (blSpeed > 3) || (brSpeed > 3);
    }

    public Rotation2d getRotation() {
        return this.m_pigeon2.getRotation2d();
    }

    public Pose2d getPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public double getOffset(int index) {
        return this.getModule(index).getCANcoder().getPosition().getValueAsDouble();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

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
}
