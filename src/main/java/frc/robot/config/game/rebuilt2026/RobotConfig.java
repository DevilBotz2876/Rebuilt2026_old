package frc.robot.config.game.rebuilt2026;

import java.util.Properties;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.io.implementations.motor.MotorIOArmStub;
import frc.robot.io.implementations.motor.MotorIOSparkMax;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOSparkMax.SparkMaxSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.controls.arm.ArmControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.interfaces.SimpleMotor;
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
      // this.autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
    }

    // Send vision-based odometry measurements to drive's odometry calculations
    // vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);
    ArmControls.setupController(arm, mainController);
    if (null != this.autoChooser) {
      SmartDashboard.putData("Autonomous", this.autoChooser);
    }
  }


  private ArmMotorSubsystem createArm(Properties robotProperties, String name) {
    ArmSettings armSettings = new ArmSettings();
    String armSettingPrefix = name + ".armSettings";

    armSettings.color = new Color8Bit(
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.red")),
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.green")),
      Integer.parseInt(robotProperties.getProperty(armSettingPrefix + ".color.blue"))
    );

    armSettings.minAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".minAngleInDegrees"));
    armSettings.maxAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxAngleInDegrees"));
    armSettings.startingAngleInDegrees = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".startingAngleInDegrees"));
    armSettings.maxVelocityInDegreesPerSecond = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxVelocityInDegreesPerSecond"));
    armSettings.maxAccelerationInDegreesPerSecondSquared = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".maxAccelerationInDegreesPerSecondSquared"));
    
    armSettings.feedforward = new ArmFeedforward(
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.ks")),
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.kg")),
      Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".feedforward.kv"))
    );

    armSettings.motor = getDCMotor(robotProperties.getProperty(armSettingPrefix + ".DCMotor"));
    armSettings.simulateGravity = Boolean.parseBoolean(robotProperties.getProperty(armSettingPrefix + ".simulateGravity"));
    armSettings.armLengthInMeters = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".armLengthInMeters"));
    armSettings.armMassInKg = Double.parseDouble(robotProperties.getProperty(armSettingPrefix + ".armMassInKg"));


    MotorIOBaseSettings IOSettings = new MotorIOBaseSettings();
    String IOSettingPrefix = name + ".IOSetting";

    IOSettings.motor.inverted = Boolean.parseBoolean(robotProperties.getProperty(IOSettingPrefix + ".inverted"));
    IOSettings.motor.gearing = Double.parseDouble(robotProperties.getProperty(IOSettingPrefix + ".gearing"));
    IOSettings.motor.drumRadiusMeters = Double.parseDouble(robotProperties.getProperty(IOSettingPrefix + ".drumRadiusMeters"));

    IOSettings.pid = new PIDController(
      Double.parseDouble(robotProperties.getProperty(IOSettingPrefix + ".pid.kp")),
      Double.parseDouble(robotProperties.getProperty(IOSettingPrefix + ".pid.ki")),
      Double.parseDouble(robotProperties.getProperty(IOSettingPrefix + ".pid.kp"))
    );

    switch (robotProperties.getProperty(name + ".motor.motorController")) {
      case "talonFX":
        TalonFxSettings talonSettings = new TalonFxSettings();
        talonSettings.canId = Integer.parseInt(robotProperties.getProperty(name + ".talonFX.setting.id"));
        return new ArmMotorSubsystem(new MotorIOTalonFx(IOSettings, talonSettings), name, armSettings);

      case "sparkMax":
        SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
        sparkMaxSettings.canId = Integer.parseInt(robotProperties.getProperty(name + ".sparkMax.setting.id"));
        return new ArmMotorSubsystem(new MotorIOSparkMax(IOSettings, sparkMaxSettings), name, armSettings);

      case "sim":
      default:
        return new ArmMotorSubsystem(new MotorIOArmStub(IOSettings, armSettings), name, armSettings);
    }
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
