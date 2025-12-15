package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.subsystems.interfaces.Drive;

public class DriveControls {
  public static void setupController(Drive drive, CommandXboxController controller) {
    SubsystemBase driveSubsystem = (SubsystemBase) drive;
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05), // Robot Strafe Front/Back
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05))); // Robot Rotate
    // () -> MathUtil.applyDeadband(-Math.signum(controller.getLeftY()) *
    // SmartDashboard.getNumber("Temp/axisValue", 0.0) * (controller.getLeftY() < 0.05 &&
    // controller.getLeftY() > -0.05 ? 0 : 1), 0.05), // Robot Strafe Front/Back
    // () -> MathUtil.applyDeadband(-Math.signum(controller.getLeftX()) *
    // SmartDashboard.getNumber("Temp/axisValue", 0.0) * (controller.getLeftX() < 0.05 &&
    // controller.getLeftX() > -0.05 ? 0 : 1), 0.05), // Robot Strafe Left/Right
    // () -> MathUtil.applyDeadband(-Math.signum(controller.getRightX()) *
    // SmartDashboard.getNumber("Temp/axisValue", 0.0) * (controller.getRightX() < 0.05 &&
    // controller.getRightX() > -0.05 ? 0 : 1), 0.05))); // Robot Rotate

    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *
     *    Start Button = Toggle Drive Orientation
     */
    controller.back().onTrue(new InstantCommand(() -> drive.resetOdometry()));
    controller
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drive.setFieldOrientedDrive(
                        !drive.isFieldOrientedDrive()))); // Toggle Drive Orientation

    // set speeds
    //     SmartDashboard.putNumber("Temp/axisValue", 1.0);
    //     controller
    //             .b()
    //             .onTrue(
    //                 new InstantCommand(() -> {
    //                     SmartDashboard.putNumber("Temp/axisValue", .25);
    //                 }
    //                 )
    //             );
    //             controller
    //             .y()
    //             .onTrue(
    //                 new InstantCommand(() -> {
    //                     SmartDashboard.putNumber("Temp/axisValue", .5);
    //                 }
    //                 )
    //             );

    //             controller
    //             .x()
    //             .onTrue(
    //                 new InstantCommand(() -> {
    //                     SmartDashboard.putNumber("Temp/axisValue", .75);
    //                 }
    //                 )
    //             );

    //             controller
    //             .a()
    //             .onTrue(
    //                 new InstantCommand(() -> {
    //                     SmartDashboard.putNumber("Temp/axisValue", 1.0);
    //                 }
    //                 )
    //             );
  }
}
