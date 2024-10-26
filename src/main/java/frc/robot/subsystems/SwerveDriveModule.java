package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveDriveModule {
    private final TalonFX WheelMotor;
    private final TalonFX ControllerMotor;
    private final CANcoder Encoder;

    private double rotationSetpoint = 0.0;
    private double speed = 0.0;

    private double encoderOffset = 0.0;

    private boolean inverted = false;

    ProfiledPIDController PID = new ProfiledPIDController(
        0.01, 
        0.0, 
        0.0, 
        new TrapezoidProfile.Constraints(0.8, 0.5)
    );

    public SwerveDriveModule(int wheelID, int controllerID, int encoderID){
        WheelMotor = new TalonFX(wheelID);
        ControllerMotor = new TalonFX(controllerID);
        Encoder = new CANcoder(encoderID);

        PID.enableContinuousInput(0, 1);
    }

    public double getPosition(){
        return inverted ? -1 : 1 * (Encoder.getAbsolutePosition().getValue() - encoderOffset);
    }

    public void periodic(){
        WheelMotor.set(speed);
        ControllerMotor.set(PID.calculate(getPosition(), rotationSetpoint));
    } 

    public void setSwerveModuleState(SwerveModuleState state){
        SwerveModuleState s = SwerveModuleState.optimize(state, Rotation2d.fromRotations(getPosition()));
        rotationSetpoint = s.angle.getRotations();
        speed = toWheelVelocity(toRotationsPerSecond(speed));
    }

    public SwerveModuleState toSwerveModuleState(){
        return new SwerveModuleState(getMetersPerSecondSpeed(), Rotation2d.fromRotations(getPosition()));
    }

    public double getMetersPerSecondSpeed(){
        return toWheelMetersPerSecond(WheelMotor.getVelocity().getValue());
    }

    public static double toWheelMetersPerSecond(double rotationsPerSecond){
        double GearRatio = (1/6.12);
        double WheelRPS = rotationsPerSecond*GearRatio;
        double WheelDiameter = (3.75/39.37);
        return WheelRPS*Math.PI*WheelDiameter;
    }

    public static double toRotationsPerSecond(double metersPerSecond){
        double GearRatio = (6.12/1);
        double WheelDiameter = (3.75/39.37);
        double WheelRPS = metersPerSecond/(Math.PI*WheelDiameter);
        return WheelRPS*GearRatio;
    }

    public static double toWheelVelocity(double rotationsPerSecond){
        return rotationsPerSecond/512.0;
    }

    public void setInverted(boolean b){
        inverted = b;
        WheelMotor.setInverted(b);
        ControllerMotor.setInverted(b);
    }

    public ProfiledPIDController getPID(){
        return PID;
    }
}
