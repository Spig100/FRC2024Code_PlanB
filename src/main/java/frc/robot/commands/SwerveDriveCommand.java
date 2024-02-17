package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerSubsystem;


public class SwerveDriveCommand extends Command {
    private final DriveSubsystem drive = DriveSubsystem.getInstance();
    private final XboxController driveController;
    private final PIDController visionAimPID = new PIDController(Constants.VISION_AIM_KP, Constants.VISION_AIM_KI, Constants.VISION_AIM_KD);

    public SwerveDriveCommand(XboxController driveController) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drive);
        this.driveController = driveController;
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(-driveController.getLeftY(), Constants.CHASSIS_DEAD_ZONE);
        double ySpeed = MathUtil.applyDeadband(-driveController.getLeftX(), Constants.CHASSIS_DEAD_ZONE);
        double zRot = MathUtil.applyDeadband(-driveController.getRightX(), Constants.CHASSIS_DEAD_ZONE);
        boolean autoAim = driveController.getXButton();
        if(autoAim && LimeLightHelpers.getTV(Constants.LIMELIGHT_NAME)) {
            double tx = LimeLightHelpers.getTX(Constants.LIMELIGHT_NAME);
            zRot += visionAimPID.calculate(tx, 0);
        }
        xSpeed *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND;
        ySpeed *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND;
        zRot *= Constants.CHASSIS_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        drive.drive(xSpeed, ySpeed, zRot, Constants.CHASSIS_ENABLE_FIELD_ORIENTED_CONTROL);
        HangerSubsystem.getInstance().setHangerSpeed(driveController.getRightTriggerAxis()-driveController.getLeftTriggerAxis());
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopAll();
    }
}
