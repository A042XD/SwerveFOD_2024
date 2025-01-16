// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

/*
  * XBox Controller Key Binds
  * 
  *  - Left Joystick: Move (8 Directional - South, West, North, East, Northeast, ...) (Field Relative/Field Oriented)
  *  - Right Joystick: Rotate Bot
  *  - B: Reset gyroscope
  * 
  */

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    XboxController driver = new XboxController(0);
    public final JoystickButton driverX = new JoystickButton(driver, 2);
    public final SwerveSubsystem swerveSub = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/config"));
    public RobotContainer() {
        configureBindings();

        FieldOrientedDrive xBoxFODrive = new FieldOrientedDrive(
          swerveSub,
          () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-driver.getRightX(), OperatorConstants.ROTATION_DEADBAND),
          () -> true);
        swerveSub.setDefaultCommand(xBoxFODrive);
    }

    private void configureBindings() {
      driverX.onTrue(Commands.runOnce(swerveSub::zeroGyro));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
