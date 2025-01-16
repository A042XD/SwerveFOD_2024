// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.SnapToDirection;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    XboxController driver = new XboxController(0);
    // Joystick rotDriver = new Joystick(1);

    JoystickButton driverA = new JoystickButton(driver, 1);
    JoystickButton driverB = new JoystickButton(driver, 2);
    JoystickButton driverX = new JoystickButton(driver, 3);
    JoystickButton driverY = new JoystickButton(driver, 4);
    JoystickButton driverLeftBumper = new JoystickButton(driver, 5);
    JoystickButton driverRightBumper = new JoystickButton(driver, 6);
    public final SwerveSubsystem swerveSub = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/config"));
    public RobotContainer() {
        configureBindings();

        FieldOrientedDrive xBoxTeleopDrive = new FieldOrientedDrive(
          swerveSub,
          () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-driver.getRightX(), OperatorConstants.ROTATION_DEADBAND),
          () -> true);
        swerveSub.setDefaultCommand(xBoxTeleopDrive);
    }

    private void configureBindings() {
      driverX.onTrue(Commands.runOnce(swerveSub::zeroGyro));
      // new Trigger(() -> driver.getPOV() != -1)
      //   .whileTrue(new SnapToDirection(swerveSub, new Rotation2d(driver.getPOV() * 3.14 / 180), () -> true));
      // driverA.whileTrue(new SnapToDirection(swerveSub, new Rotation2d(0), () -> true));
      // driverB.whileTrue(new SnapToDirection(swerveSub, new Rotation2d(90 * Math.PI / 180), () -> true));
      // driverX.whileTrue(new SnapToDirection(swerveSub, new Rotation2d(180 * Math.PI / 180), () -> true));
      // driverY.whileTrue(new SnapToDirection(swerveSub, new Rotation2d(270 * Math.PI / 180), () -> true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
