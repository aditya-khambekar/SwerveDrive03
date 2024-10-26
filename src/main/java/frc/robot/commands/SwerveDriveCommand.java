package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends Command{
    SwerveDriveSubsystem subsystem = SwerveDriveSubsystem.getInstance();
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier angularRotationSupplier;

    public SwerveDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier angularRotationSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.angularRotationSupplier = angularRotationSupplier;
    }


    @Override
    public void initialize(){
        subsystem.setChassisSpeeds(new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angularRotationSupplier.getAsDouble()));
    }


}
