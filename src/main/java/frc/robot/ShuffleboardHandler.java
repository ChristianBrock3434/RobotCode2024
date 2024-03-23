package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The ShuffleboardHandler class handles the initialization of Shuffleboard tabs for sensor and driver data.
 * It provides methods to add various types of data to the Shuffleboard tabs.
 */
public class ShuffleboardHandler {
    private static ShuffleboardTab sensorTab = Shuffleboard.getTab("Sensors");

    /**
     * Initializes the Shuffleboard tab for sensor data
     */
    public static void initSensorTab () {
        sensorTab.addNumber("Gyro", drivetrain::getDegrees)
                        .withPosition(6, 1)
                        .withSize(2, 2)
                        .withWidget(BuiltInWidgets.kGyro)
                        .withProperties(Map.of("Counter Clockwise", true));
        sensorTab.addNumber("x Pose", drivetrain::getX)
                        .withPosition(1, 0);
        sensorTab.addNumber("y Pose", drivetrain::getY)
                        .withPosition(2, 0);

        sensorTab.addBoolean("Left Intake Note Sensor", intake::getLeftNoteSensor)
                        .withPosition(0, 1)
                        .withSize(2, 1);
        sensorTab.addBoolean("Right Intake Note Sensor", intake::getRightNoteSensor)
                        .withPosition(2, 1)
                        .withSize(2, 1);
        sensorTab.addNumber("Intake Speed", intake::getVelocity)
                        .withPosition(4, 1)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", 0, "max", 80));

        sensorTab.addDouble("Actuation Angle", actuation::getAngle)
                        .withPosition(5, 1)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", -65, "max", 105));
        
        sensorTab.addNumber("Angle Controller", angleController::getAngle)
                        .withPosition(5, 2)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", 0, "max", 35));

        sensorTab.addBoolean("Angle Zero", angleController::getZeroSensor)
                        .withPosition(0, 0);

        sensorTab.addNumber("Indexer Speed", indexer::getVelocity)
                        .withPosition(4, 2)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", 0, "max", 80));

        sensorTab.addBoolean("Shooter line break", shooter::getNoteSensor)
                        .withPosition(3, 0);
        sensorTab.addNumber("Left Shooter Speed", shooter::getLeftVelocity)
                        .withPosition(0, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", 0, "max", 80));
        sensorTab.addNumber("Right Shooter Speed", shooter::getLeftVelocity)
                        .withPosition(2, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kDial)
                        .withProperties(Map.of("min", 0, "max", 80));

        sensorTab.addNumber("Shooter TX", limelightShooter::getTX)
                        .withPosition(8, 0);
        sensorTab.addNumber("Shooter TY", limelightShooter::getTY)
                        .withPosition(9, 0);

        sensorTab.addNumber("Intake TX", limelightIntake::getTX)
                        .withPosition(8, 1);
        sensorTab.addNumber("Intake TY", limelightIntake::getTY)
                        .withPosition(9, 1);

        sensorTab.addNumber("Distance from goal", drivetrain::getDistanceFromGoal)
                        .withPosition(8, 2)
                        .withSize(2, 1);
    }

    private static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    private static GenericEntry FarShotRange;

    private static GenericEntry ampShotSpeed;
    private static GenericEntry FarShotSpeed;
    private static GenericEntry SubwooferSpeed;
    private static GenericEntry PodiumSpeed;

    private static GenericEntry IntakeAngle;

    /**
     * Initializes the Shuffleboard tab for driver data
     * @param autoChooser The SendableChooser for the autonomous mode
     * @param manualMode The BooleanSupplier for the manual mode
     */
    public static void initDriverTab (SendableChooser<Command> autoChooser, BooleanSupplier manualMode) {
        
        driverTab.addDouble("Far Shot Angle", () -> chainShotAngle)
                        .withPosition(4, 0)
                        .withSize(1, 1);

        driverTab.addDouble("Amp Shot Angle", () -> ampAngle)
                        .withPosition(4, 1)
                        .withSize(1, 1);

        driverTab.addDouble("Subwoofer Shot Angle", () -> subwooferShotAngle)
                        .withPosition(4, 2)
                        .withSize(1, 1);

        driverTab.addDouble("Podium Shot Angle", () -> podiumShotAngle)
                        .withPosition(4, 3)
                        .withSize(1, 1);

        driverTab.addDouble("Pass Shot Angle", () -> passShotAngle)
                        .withPosition(3, 3)
                        .withSize(1, 1);



        FarShotRange = driverTab.add("Far Shot Distance", farShotDistance)
                        .withPosition(0, 0)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", farShotDistance-3, "max", farShotDistance+3))
                        .getEntry();

        ampShotSpeed = driverTab.add("Amp Shot Speed", ampSpeed)
                        .withPosition(2, 0)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", ampSpeed-5, "max", ampSpeed+5))
                        .getEntry();

        IntakeAngle = driverTab.add("Intake Angle", actuationPickUpPosition)
                        .withPosition(0, 1)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", actuationPickUpPosition-5, "max", actuationPickUpPosition+5))
                        .getEntry();

        FarShotSpeed = driverTab.add("Far Shot Speed", chainShotSpeed)
                        .withPosition(2, 1)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", chainShotSpeed-10, "max", chainShotSpeed+10))
                        .getEntry();

        SubwooferSpeed = driverTab.add("Subwoofer Shot Speed", subwooferShotSpeed)
                        .withPosition(0, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", subwooferShotSpeed-10, "max", subwooferShotSpeed+10))
                        .getEntry();

        PodiumSpeed = driverTab.add("Podium Shot Speed", podiumShotSpeed)
                        .withPosition(2, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", podiumShotSpeed-10, "max", podiumShotSpeed+10))
                        .getEntry();

        driverTab.addBoolean("Manual Shot", manualMode)
                        .withPosition(5, 0)
                        .withSize(1, 4);

        driverTab.add("Auto Chooser", autoChooser)
                        .withPosition(0, 3)
                        .withSize(3, 1);
                    
        driverTab.addCamera("Limelight Intake", "intake", "mjpg:http://10.17.30.93:5800")
                        .withPosition(6, 0)
                        .withSize(4, 4)
                        .withProperties(Map.of("Show Controls", false));
    }

    /**
     * Updates the code with the current values in shuffleboard
     */
    public static void updateDriverTab () {
        farShotDistance = FarShotRange.getDouble(farShotDistance);
        actuationPickUpPosition = IntakeAngle.getDouble(actuationPickUpPosition);

        ampSpeed = ampShotSpeed.getDouble(ampSpeed);
        chainShotSpeed = FarShotSpeed.getDouble(chainShotSpeed);
        subwooferShotSpeed = SubwooferSpeed.getDouble(subwooferShotSpeed);
        podiumShotSpeed = PodiumSpeed.getDouble(podiumShotSpeed);
    }
}
