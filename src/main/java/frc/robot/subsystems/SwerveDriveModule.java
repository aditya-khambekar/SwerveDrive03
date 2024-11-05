package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveDriveModule{
    private TalonFX rotator;
    private TalonFX wheel;
    private CANcoder encoder;

    private PIDController pid;

    private double speed;
    private Rotation2d rotatorSetPoint;

    //sim
    private String name;
    Mechanism2d mech = new Mechanism2d(4, 4);
    MechanismRoot2d root = mech.getRoot(name, 2, 2);
    MechanismLigament2d wheelLigament = root.append(new MechanismLigament2d("wheel", 2, 0));


    public SwerveDriveModule(String name, int rotatorID, int wheelID, int encoderID){
        rotator = new TalonFX(rotatorID);
        wheel = new TalonFX(wheelID);
        encoder = new CANcoder(encoderID);

        System.out.println(name+" added");

        pid = new PIDController(
                0.01,
                0,
                0
        );

        pid.enableContinuousInput(0., 1);
        speed = 0.0;
        rotatorSetPoint = Rotation2d.fromRotations(0.0);
        this.name = name;
    }

    public void setDesiredState(SwerveModuleState s){
        SwerveModuleState optimized = SwerveModuleState.optimize(s, getActualState().angle);
        speed = optimized.speedMetersPerSecond;
        rotatorSetPoint = optimized.angle;
    }

    public Double getPosition(){
        return encoder.getAbsolutePosition().getValue();
    }

    public SwerveModuleState getActualState(){
        return new SwerveModuleState(wheel.getVelocity().getValue(), Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue()));
    }

    public void periodic(){
        rotator.set(pid.calculate(getPosition(), rotatorSetPoint.getRotations()));
        wheel.set(speed);
        wheelLigament.setAngle(true ? rotatorSetPoint.getDegrees() : degreeOpposite(rotatorSetPoint.getDegrees()));
        wheelLigament.setColor(speed > 0 ? new Color8Bit(Color.kAliceBlue) : new Color8Bit(Color.kRed));
        SmartDashboard.putData(name, mech);
    }

    public void setPID(double kp, double ki, double kd){
        pid.setPID(kp, ki, kd);
    }

    public static double degreeOpposite(double degree){
        return 360-degree;
    }
}