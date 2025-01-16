package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class FieldOrientedDrive extends Command{
    private final SwerveSubsystem swerveSub;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public FieldOrientedDrive(SwerveSubsystem swerveSub, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode){
        this.swerveSub = swerveSub;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = swerveSub.getSwerveController();
        addRequirements(swerveSub);
    }
    @Override
    public void execute(){
        double xv = Math.pow(vX.getAsDouble(), 3);
        double yv = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);

        // Field Oriented
        double rot = -(360 - swerveSub.getYaw().getDegrees()) * 3.14 / 180;
        // double forward = yv;
        // double strafe = xv;
        double forward = yv * Math.cos(rot) - xv * Math.sin(rot);
        double strafe = yv * Math.sin(rot) + xv * Math.cos(rot);
        forward = forward * swerveSub.maximumSpeed;
        strafe = strafe * swerveSub.maximumSpeed;
        
        // Publish Data
        NetworkTableInstance.getDefault().getTable("Dashboard").getEntry("angVelocity").setDouble(angVelocity);
        NetworkTableInstance.getDefault().getTable("Dashboard").getEntry("rot").setDouble(rot);
        NetworkTableInstance.getDefault().getTable("Dashboard").getEntry("cos").setDouble(Math.cos(rot));

        // Drive
        Translation2d trans = new Translation2d(strafe, forward);
        swerveSub.drive(trans, angVelocity * controller.config.maxAngularVelocity, driveMode.getAsBoolean());
    }
}
