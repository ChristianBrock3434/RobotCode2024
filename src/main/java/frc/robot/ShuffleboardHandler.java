package frc.robot;

import static frc.robot.Constants.farShotDistance;
import static frc.robot.Constants.ActuationConstants.actuationPickUpPosition;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Subsystems.*;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardHandler {
    private static ShuffleboardTab sensorTab = Shuffleboard.getTab("Sensors");

    /**
     * Initializes the Shuffleboard tab for sensor data
     */
    public static void initSensorTab () {
        sensorTab.addNumber("Gyro", drivetrain::getDegrees)
                        .withPosition(6, 1)
                        .withSize(2, 2)
                        .withWidget(BuiltInWidgets.kGyro);
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

        sensorTab.addNumber("Distance from goal", limelightShooter::getDistanceFromGoal)
                        .withPosition(8, 2)
                        .withSize(2, 1);
    }

    private static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    private static GenericEntry FarShotAngle;
    private static GenericEntry FarShotRange;

    private static GenericEntry PodiumShotAngle;
    private static GenericEntry SubwooferShotAngle;
    private static GenericEntry PassShotAngle;

    private static GenericEntry IntakeAngle;

    /**
     * Initializes the Shuffleboard tab for driver data
     * @param autoChooser The SendableChooser for the autonomous mode
     * @param manualMode The BooleanSupplier for the manual mode
     */
    public static void initDriverTab (SendableChooser<String> autoChooser, BooleanSupplier manualMode) {
        // test = driverTab.add("Test", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        FarShotAngle = driverTab.add("Far Shot Angle", chainShotAngle)
                        .withPosition(0, 0)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", chainShotAngle-3, "max", chainShotAngle+3))
                        .getEntry();

        FarShotRange = driverTab.add("Far Shot Distance", farShotDistance)
                        .withPosition(2, 0)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", farShotDistance-3, "max", farShotDistance+3))
                        .getEntry();

        PodiumShotAngle = driverTab.add("Podium Shot Angle", podiumShotAngle)
                        .withPosition(0, 1)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", podiumShotAngle-3, "max", podiumShotAngle+3))
                        .getEntry();

        SubwooferShotAngle = driverTab.add("Subwoofer Shot Angle", subwooferShotAngle)
                        .withPosition(2, 1)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", 0, "max", subwooferShotAngle+5))
                        .getEntry();

        PassShotAngle = driverTab.add("Pass Shot Angle", passShotAngle)
                        .withPosition(2, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", passShotAngle-5, "max", passShotAngle+5))
                        .getEntry();

        IntakeAngle = driverTab.add("Intake Angle", actuationPickUpPosition)
                        .withPosition(0, 2)
                        .withSize(2, 1)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", actuationPickUpPosition-5, "max", actuationPickUpPosition+5))
                        .getEntry();

        driverTab.addBoolean("Manual Shot", manualMode)
                        .withPosition(4, 0)
                        .withSize(1, 4);

        driverTab.add("Auto Chooser", autoChooser)
                        .withPosition(0, 3)
                        .withSize(4, 1);
                    
        driverTab.addCamera("Limelight Shooter", "shooter", "mjpg:http://10.17.30.213:5800")
                        .withPosition(5, 0)
                        .withSize(5, 5)
                        .withProperties(Map.of("Show Controls", false));
    }

    /**
     * Updates the code with the current values in shuffleboard
     */
    public static void updateDriverTab () {
        chainShotAngle = FarShotAngle.getDouble(chainShotAngle);
        farShotDistance = FarShotRange.getDouble(farShotDistance);
        podiumShotAngle = PodiumShotAngle.getDouble(podiumShotAngle);
        subwooferShotAngle = SubwooferShotAngle.getDouble(subwooferShotAngle);
        passShotAngle = PassShotAngle.getDouble(passShotAngle);
        actuationPickUpPosition = IntakeAngle.getDouble(actuationPickUpPosition);
    }
}
