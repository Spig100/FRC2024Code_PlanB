// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final double CHASSIS_WIDTH_METERS = 0.62865;
    public static final double CHASSIS_LENGTH_METERS = 0.62865;
    public static final double CHASSIS_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final DriveSubsystem.GearRatio CHASSIS_DRIVE_GEAR_RATIO = DriveSubsystem.GearRatio.L1;
    public static final double CHASSIS_ANGLE_GEAR_RATIO = 21.42;
    public static final double CHASSIS_MAX_SPEED_METERS_PER_SECOND = 8.0;
    public static final double CHASSIS_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 6.0;
    public static final double CHASSIS_MAX_ACCELERATION_METERS_PER_SECOND = 5.0;
    public static final boolean CHASSIS_ENABLE_FIELD_ORIENTED_CONTROL = true;
    public static final double CHASSIS_DEAD_ZONE = 0.1;
    public static final Pose2d CHASSIS_INITIAL_POSITION = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // TODO: use real start position
    public static final int CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_CANCODER_ID = 1;
    public static final double CHASSIS_FRONT_LEFT_ANGLE_OFFSET_DEGREES = -0.119141;
    public static final int CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_CANCODER_ID = 4;
    public static final double CHASSIS_FRONT_RIGHT_ANGLE_OFFSET_DEGREES = -0.696045;
    public static final int CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_CANCODER_ID = 2;
    public static final double CHASSIS_BACK_LEFT_ANGLE_OFFSET_DEGREES = -0.740749+0.5;
    public static final int CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_CANCODER_ID = 3;
    public static final double CHASSIS_BACK_RIGHT_ANGLE_OFFSET_DEGREES = -0.282471;
    public static final double CHASSIS_DRIVE_MOTOR_KP = 0.05;
    public static final double CHASSIS_DRIVE_MOTOR_KD = 0.0;
    public static final double CHASSIS_DRIVE_MOTOR_KS = 0.077618 / 12;
    public static final double CHASSIS_DRIVE_MOTOR_KV = 0.89851 / 12;
    public static final double CHASSIS_DRIVE_MOTOR_KA = 0.18239 / 12;
    public static final double CHASSIS_ANGLE_MOTOR_KP = 0.0004;
    public static final double CHASSIS_ANGLE_MOTOR_KD = 0.0;
    public static final double CHASSIS_ANGLE_MOTOR_KF = 0.00005;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY = 5760;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION = 10000;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 0.01;

    public static final int SHOOTER_ANGLE_MOTOR_1_ID = 10;
    public static final boolean SHOOTER_ANGLE_MOTOR_1_INVERTED = true;
    public static final int SHOOTER_ANGLE_MOTOR_2_ID = 11;
    public static final boolean SHOOTER_ANGLE_MOTOR_2_INVERTED = false;
    public static final double SHOOTER_ANGLE_MOTOR_KP = 0.00005;
    public static final double SHOOTER_ANGLE_MOTOR_KD = 0;
    public static final double SHOOTER_ANGLE_MOTOR_KF = 0.012;
    public static final double SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY = 600;
    public static final double SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION = 400;
    public static final double SHOOTER_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 3;
    public static final int SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT = 30;
    public static final double SHOOTER_ANGLE_GEAR_RATIO = 45.0 / 14 * 45 / 14 * 192 / 14 / 360; // convert to 360 degree

    public static final int SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT_ZEROING = 20;
    public static final float SHOOTER_ANGLE_MAX = 85;
    public static final int SHOOTER_FLYWHEEL_MOTOR_ID = 6;
    public static final boolean SHOOTER_FLYWHEEL_MOTOR_INVERTED = true;
    public static final double SHOOTER_FLYWHEEL_MOTOR_KP = 0.001;
    public static final double SHOOTER_FLYWHEEL_MOTOR_KD = 0;
    public static final double SHOOTER_FLYWHEEL_MOTOR_KF = 0.0127;
    public static final int SHOOTER_FLYWHEEL_MOTOR_CURRENT_LIMIT = 35;

    public static final int COLLECTOR_INTAKE_MOTOR_ID = 5;
    public static final boolean COLLECTOR_INTAKE_MOTOR_INVERTED = true;
    public static final double COLLECTOR_INTAKE_MOTOR_KP = 0.005;
    public static final double COLLECTOR_INTAKE_MOTOR_KD = 0;
    public static final double COLLECTOR_INTAKE_MOTOR_KF = 0.00227272727;
    public static final int COLLECTOR_INTAKE_MOTOR_CURRENT_LIMIT = 35;
    public static final double COLLECTOR_INTAKE_MOTOR_GEAR_RATIO = 21.0;


    public static final int COLLECTOR_TRANSFER_MOTOR_ID = 6;
    public static final boolean COLLECTOR_TRANSFER_MOTOR_INVERTED = false;
    public static final double COLLECTOR_TRANSFER_MOTOR_KP = 0.001;
    public static final double COLLECTOR_TRANSFER_MOTOR_KD = 0;
    public static final double COLLECTOR_TRANSFER_MOTOR_KF = 0.00208333333;
    public static final int COLLECTOR_TRANSFER_MOTOR_CURRENT_LIMIT = 35;
    public static final double COLLECTOR_TRANSFER_MOTOR_GEAR_RATIO = 21.0;


    public static final int COLLECTOR_SLIDER_MOTOR_ID = 7;
    public static final boolean COLLECTOR_SLIDER_MOTOR_INVERTED = true;
    public static final double COLLECTOR_SLIDER_MOTOR_KP = 0.0015;
    public static final double COLLECTOR_SLIDER_MOTOR_KD = 0;
    public static final double COLLECTOR_SLIDER_MOTOR_KF = 0.13;
    public static final int COLLECTOR_SLIDER_MOTOR_CURRENT_LIMIT = 35;
    public static final int COLLECTOR_SLIDER_MOTOR_CURRENT_LIMIT_ZEROING = 20;
    public static final double COLLECTOR_SLIDER_MOTOR_GEAR_RATIO = 7.0;
    public static final double SLIDER_MAXIMUM_LENGTH = 2.4;
    public static final double SLIDER_MINIMUM_LENGTH = 0;
    public static final double COLLECTOR_SLIDER_MOTOR_SMART_MOTION_MAX_VELOCITY = 7;
    public static final double COLLECTOR_SLIDER_MOTOR_SMART_MOTION_MAX_ACCELERATION = 5;
    public static final double COLLECTOR_SLIDER_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 0.1;

    public static final int HANGER_MOTOR_1_ID = 8;
    public static final int HANGER_MOTOR_2_ID = 9;
    public static final boolean HANGER_MOTOR_1_INVERTED = false;
    public static final boolean HANGER_MOTOR_2_INVERTED = true;
    public static final double HANGER_MOTOR_KP = 0;
    public static final double HANGER_MOTOR_KD = 0;
    public static final double HANGER_MOTOR_KF = 0;
    public static final double HANGER_MOTOR_SMART_MOTION_MAX_VELOCITY = 1000;
    public static final double HANGER_MOTOR_SMART_MOTION_MAX_ACCELERATION = 1000;
    public static final double HANGER_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 0.1;
    public static final int HANGER_MOTOR_CURRENT_LIMIT = 38;
    public static final int HANGER_MOTOR_CURRENT_LIMIT_ZEROING = 30;
    public static final double HANGER_MOTOR_GEAR_RATIO = 50;
    public static final double HANGER_MAX_LENGTH = 300; // cm

    public static final String LIMELIGHT_NAME = "limelight";
    public static final double VISION_AIM_KP = 0.01;
    public static final double VISION_AIM_KI = 0.0;
    public static final double VISION_AIM_KD = 0.0;
}
