package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class SnapToDirection extends Command{
    private final SwerveSubsystem swerveSub;
    private final BooleanSupplier driveMode;
    private Rotation2d dir;
    private SwerveController controller;

    public SnapToDirection(SwerveSubsystem swerveSub, Rotation2d dir, BooleanSupplier driveMode){
        this.swerveSub = swerveSub;
        this.dir = dir;
        this.driveMode = driveMode;
        if (this.dir.getRadians() < 0){
            this.dir = new Rotation2d(this.dir.getRadians() + 2 * Math.PI);
        }
        while (this.dir.getRadians() > 2 * Math.PI){
            this.dir = new Rotation2d(this.dir.getRadians() - 2 * Math.PI);
        }
        this.controller = swerveSub.getSwerveController();
    }

    @Override
    public void execute(){
        double x = dir.getRadians();
        double y = swerveSub.getYaw().getRadians();
        double targ = (y - x) / 2.0;
        if (y > x){
            targ = Math.min(y - x, controller.config.maxAngularVelocity);
        }
        else{
            targ = y - x;
        }
        if (Math.abs(targ) / 2.0 > 0.1)
            swerveSub.drive(new Translation2d(), targ / 2.0, driveMode.getAsBoolean());
    }
}
