// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.subsystems.SwerveDriveSubsystem;



public class RobotContainer
{

    OI oi = new OI();
    public RobotContainer()
    {
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
                        var rpm = 512*x;
                        return SwerveDriveModule.toWheelMetersPerSecond(rpm);
                    }
                    
                }, 
                //ySupplier
                new DoubleSupplier() {

                    @Override
                    public double getAsDouble() {
                        var y = oi.driverController().getAxis(OI.Axes.RIGHT_STICK_Y);
                        var rpm = 512*y;
                        return SwerveDriveModule.toWheelMetersPerSecond(rpm);
                    }
                    
                }, 
                //angularRotationSupplier
                new DoubleSupplier() {

                    @Override
                    public double getAsDouble() {
                        var x = oi.driverController().getAxis(OI.Axes.LEFT_STICK_X);
                        var y = oi.driverController().getAxis(OI.Axes.LEFT_STICK_Y);

                        //this gets the angle from 0 to 2pi and subtracts pi to make it range from pi to negative pi
                        var angle = Math.atan2(y,x)-Math.PI;
                        return angle;
                    }
                    
                })
        );
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
