// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.SwerveDriveSubsystem;



public class RobotContainer
{

    SwerveDriveSubsystem swerveDriveSubsystem = SwerveDriveSubsystem.getInstance();
    OI oi = new OI();
    public RobotContainer()
    {
        System.out.println("Robot Container Init");
        configureBindings();
    }
    
    
    private void configureBindings() {
        SwerveDriveSubsystem.getInstance().setDefaultCommand(
            new SwerveDriveCommand(
                //xSupplier
                new DoubleSupplier() {

                    @Override
                    public double getAsDouble() {
                        var x = oi.driverController().getAxis(OI.Axes.RIGHT_STICK_X);
                        //System.out.println("X: "+x);
                        return x;
                    }
                    
                }, 
                //ySupplier
                new DoubleSupplier() {

                    @Override
                    public double getAsDouble() {
                        var y = oi.driverController().getAxis(OI.Axes.RIGHT_STICK_Y);
                        //System.out.println("Y: "+y);
                        return y;
                    }
                    
                }, 
                //angularRotationSupplier
                new DoubleSupplier() {

                    @Override
                    public double getAsDouble() {
                        var x = oi.driverController().getAxis(OI.Axes.LEFT_STICK_X);
                        
                        var angle = -Math.PI*x;
                        //System.out.println("rotation = "+angle);
                        return angle;
                    }
                    
                })
        );
        oi.driverController().getButton(OI.Buttons.A_BUTTON).whileTrue(new Command(){
            @Override
            public void execute() {
                System.out.println("A Button");
            }
        });
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
