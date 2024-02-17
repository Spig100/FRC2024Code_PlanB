// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

    double angle = 0;

    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
        SwerveDriveCommand driveCommand = new SwerveDriveCommand(driverController);
        driveSubsystem.setDefaultCommand(driveCommand);
        AutoBuilder.configureHolonomic(
                driveSubsystem::getPose, // Pose2d supplier
                driveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
                driveSubsystem::getChassisSpeeds, // SwerveDriveKinematics
                driveSubsystem::setChassisSpeeds, // Module states consumer used to output to the drive subsystem
                new HolonomicPathFollowerConfig(
                        new PIDConstants(1, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                        new PIDConstants(0.1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                        1, // The maximum velocity of the robot. Optional, defaults to 1
                        Constants.CHASSIS_WIDTH_METERS / 2,
                        new ReplanningConfig()
                ), // The path constraints. Optional, defaults to null
                () -> true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(
                () -> {
                    CollectSubsystem.getInstance().enableIntakeMotor();
                }
        ));
        new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(
                () -> {
                    CollectSubsystem.getInstance().disableIntakeMotor();
                }
        ));
        new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            ShooterSubsystem.getInstance().setAngle(70);
                            ShooterSubsystem.getInstance().setFlywheel(6000);
                        }
                ),
                new WaitCommand(7),
                new InstantCommand(() -> {
                    CollectSubsystem.getInstance().setShooting(true);
                }),
                new WaitCommand(2),
                new InstantCommand(() -> {
                    CollectSubsystem.getInstance().setShooting(false);
                    ShooterSubsystem.getInstance().setFlywheel(0);
                    ShooterSubsystem.getInstance().setAngle(0);
                })
        ));
//        new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(armSubsystem::raiseArm));
//        new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(armSubsystem::lowerArm));
//        new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(armSubsystem::closeClaw));
//        new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(armSubsystem::openClaw));
//        new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(armSubsystem::testLower));
//        new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(armSubsystem::testRaise));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return AutoBuilder.buildAuto("Path 1");
        //return new AutoBalanceCommand(false);
    }
}
