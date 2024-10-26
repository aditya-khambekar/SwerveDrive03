package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Objects;
import java.util.stream.IntStream;

import com.fasterxml.jackson.databind.ser.std.MapProperty;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase{
    private static volatile SwerveDriveSubsystem instance;
    private SwerveDriveKinematics kinematics;

    SwerveDriveModule[][] modules = new SwerveDriveModule[][]{
        {new SwerveDriveModule(Constants.WHEEL1, Constants.CONTROLLER1,Constants.ENCODER1), new SwerveDriveModule(Constants.WHEEL2, Constants.CONTROLLER2, Constants.ENCODER2)}, 
        {new SwerveDriveModule(Constants.WHEEL3, Constants.CONTROLLER3,Constants.ENCODER3), new SwerveDriveModule(Constants.WHEEL4, Constants.CONTROLLER4, Constants.ENCODER4)}
    };

    public static synchronized SwerveDriveSubsystem getInstance(){
        return instance = Objects.requireNonNullElseGet(instance, SwerveDriveSubsystem::new);
    }

    private SwerveDriveSubsystem(){
        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.3048, 0.3048),
            new Translation2d(0.3048, -0.3048),
            new Translation2d(-0.3048, 0.3048),
            new Translation2d(-0.3048, -0.3048)
        );

        modules[0][1].setInverted(true);
        modules[1][1].setInverted(true);

        //TODO: add offsets
        modules[0][0].setEncoderOffset(0);
        modules[0][1].setEncoderOffset(0);
        modules[1][0].setEncoderOffset(0);
        modules[1][1].setEncoderOffset(0);

    }

    public void setChassisSpeeds(ChassisSpeeds s){
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(s);
        SwerveModuleState[][] _ModuleStates = new SwerveModuleState[][]{
            {moduleStates[0], moduleStates[1]},
            {moduleStates[2], moduleStates[3]}
        };
        IntStream.range(0, _ModuleStates.length).forEach(i ->
            IntStream.range(0, _ModuleStates[0].length).forEach(j ->
                modules[i][j].setSwerveModuleState(_ModuleStates[i][j])
            )
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("LF Desired", modules[0][0].getPID().getGoal().position);
        for(SwerveDriveModule[] m:modules){
            for(SwerveDriveModule module:m){
                SmartDashboard.putNumber(Arrays.toString(getPosition(module)), module.getPosition());
                module.periodic();
            }
        }
    }

    public int[] getPosition(SwerveDriveModule s){
        for(int i = 0; i<modules.length; i++){
            for(int j = 0; j<modules[0].length; j++){
                if(modules[i][j]==s){
                    return new int[]{i, j};
                }
            }
        }
        return new int[]{-1, -1};
    }



    
}
