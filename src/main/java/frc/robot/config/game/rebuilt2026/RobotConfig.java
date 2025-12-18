package frc.robot.config.game.rebuilt2026;

import java.util.Properties;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public DriveBase drive;
  public ArmMotorSubsystem arm;
  public SendableChooser<Command> autoChooser;
  // TODO: Add VisionSubsystem Declaration

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(Properties robotProperties) {
    arm = createArm(robotProperties, "myArm");
  }

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    if (stubDrive) {
      drive = new DriveBase("Stub");
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    // TODO: Add VisionSubsystem Initialization

    if (stubVision) {
      // TODO: Add VisionSubsystem Settings
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      // TODO: Add VisionSubsystem Simulation Support

      // HACK just to verify autos are visible without connecting to robot
      this.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }

    // Send vision-based odometry measurements to drive's odometry calculations
    // vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    if (null != this.autoChooser) {
      SmartDashboard.putData("Autonomous", this.autoChooser);
    }
  }


  private ArmMotorSubsystem createArm(Properties robotProperties, String name) {
    ArmSettings settings = new ArmSettings();
    String armSettingPrefix = name + ".settings";
    settings.color = new Color8Bit(
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.red", "0")),
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.green", "0")),
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.blue", "0"))
    );

    settings.minAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".minAngleInDegrees", "0"));
    settings.maxAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxAngleInDegrees", "0"));
    settings.startingAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".startingAngleInDegrees", "0"));
    settings.maxVelocityInDegreesPerSecond = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxVelocityInDegreesPerSecond", "0"));
    settings.maxAccelerationInDegreesPerSecondSquared = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxAccelerationInDegreesPerSecondSquared", "0"));
    settings.feedforward = new ArmFeedforward(
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.ks", "0")),
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.kg", "0")),
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.kv", "0"))
    );

    settings.motor = getDCMotor(robotProperties.getProperty(armSettingPrefix + ".DCMotor", "UNKOWN"));
    settings.simulateGravity = Boolean.parseBoolean(robotProperties.getProperty(armSettingPrefix + ".simulateGravity", "true"));
    settings.armLengthInMeters = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".armLengthInMeters", "0.0"));
    settings.armMassInKg = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".armMassInKg", "0.0"));
    return new ArmMotorSubsystem(null, name, null);
  }

  private DCMotor getDCMotor(String motor) {
    switch (motor) {
      case "KrakenX60":
        return DCMotor.getKrakenX60(1);
      // more motors if needed
      default:
        return DCMotor.getKrakenX60(1);
    }
  }
}
