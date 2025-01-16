package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveDrive drive;
    public double maximumSpeed = Units.feetToMeters(14.5);

    public SwerveSubsystem(File dir){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try{
            drive = new SwerveParser(dir).createSwerveDrive(maximumSpeed);
        }
        catch (Exception e){
            throw new RuntimeException(e);
        }
        drive.setHeadingCorrection(false);
        for(swervelib.SwerveModule module : drive.getModules())
            module.setAntiJitter(false);
        drive.setCosineCompensator(false);
        drive.setAngularVelocityCompensation(true, true, 0.1);
        drive.setModuleEncoderAutoSynchronize(false, 1);
        drive.pushOffsetsToEncoders();
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }  
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier andularRotX){
        return run(() -> {
            drive.drive(SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * drive.getMaximumVelocity(),
                translationY.getAsDouble() * drive.getMaximumVelocity()),
                0.8), 
            Math.pow(andularRotX.getAsDouble(), 3) * drive.getMaximumAngularVelocity(),
            true,
            false);
        });
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
        drive.drive(translation, rotation, fieldRelative, false);
    }
    public void zeroGyro(){
        drive.zeroGyro();
    }
    public Rotation2d getPitch(){
        return drive.getPitch();
    }
    public Rotation2d getYaw(){
        return drive.getYaw();
    }
    public Rotation2d getRoll(){
        return drive.getRoll();
    }
    public ChassisSpeeds getRobotVelocity(){
        return drive.getRobotVelocity();
    }
    public SwerveController getSwerveController(){
        return drive.swerveController;
    }
}
