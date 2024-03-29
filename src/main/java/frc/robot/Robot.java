// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;

    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        DriveSubsystem.getInstance().stopAll();
    }
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
        DriveSubsystem.getInstance().syncPositions();
    }
    
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        DriveSubsystem.getInstance().navX.zeroYaw();
        DriveSubsystem.getInstance().syncPositions();
        ShooterSubsystem.getInstance().zeroAngle();
        CollectSubsystem.getInstance().zeroSlider();
        HangerSubsystem.getInstance().zeroHanger();
    }
    
    @Override
    public void teleopPeriodic() {
        if (CollectSubsystem.getInstance().getIRSensor()){
            ShooterSubsystem.getInstance().setAngle(70);
        }else{
            ShooterSubsystem.getInstance().setAngle(0);
        }

    }

    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        DriveSubsystem.getInstance().syncPositions();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        super.simulationInit();
    }

    @Override
    public void simulationPeriodic() {}
}
