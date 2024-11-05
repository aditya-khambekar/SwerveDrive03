package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Objects;

public class SwerveDriveSubsystem extends SubsystemBase {
    private static volatile SwerveDriveSubsystem instance;
    public SwerveDriveModule
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight;

    SwerveDriveKinematics m_kinematics;

    public static synchronized SwerveDriveSubsystem getInstance(){
        return instance = Objects.requireNonNullElseGet(instance, SwerveDriveSubsystem::new);
    }

    private SwerveDriveSubsystem(){
        FrontLeft = new SwerveDriveModule("FrontLeft", Constants.CONTROLLER1, Constants.WHEEL1, Constants.ENCODER1);
        FrontRight = new SwerveDriveModule("FrontRight", Constants.CONTROLLER2, Constants.WHEEL2, Constants.ENCODER2);
        BackLeft = new SwerveDriveModule("BackLeft", Constants.CONTROLLER3, Constants.WHEEL3, Constants.ENCODER3);
        BackRight = new SwerveDriveModule("BackRight", Constants.CONTROLLER4, Constants.WHEEL4, Constants.ENCODER4);

        m_kinematics = new SwerveDriveKinematics(
                new Translation2d(-Constants.WHEELBASE_M/2, Constants.TRACKWIDTH_M/2), new Translation2d(Constants.WHEELBASE_M/2, Constants.TRACKWIDTH_M/2),
                new Translation2d(-Constants.WHEELBASE_M/2, -Constants.TRACKWIDTH_M/2), new Translation2d(Constants.WHEELBASE_M/2, -Constants.TRACKWIDTH_M/2)
        );
    }

    public void stop(){
        FrontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRotations(0.0)));
        FrontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRotations(0.0)));
        BackLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRotations(0.0)));
        BackRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRotations(0.0)));
    }

    public void setChassisSpeeds(ChassisSpeeds s){
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(s);
        FrontLeft.setDesiredState(states[0]);
        FrontRight.setDesiredState(states[1]);
        BackLeft.setDesiredState(states[2]);
        BackRight.setDesiredState(states[3]);
    }

    public void periodic(){
        FrontLeft.periodic();
        FrontRight.periodic();
        BackLeft.periodic();
        BackRight.periodic();
    }
}