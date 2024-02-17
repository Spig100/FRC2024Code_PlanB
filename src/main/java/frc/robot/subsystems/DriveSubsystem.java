package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DriveSubsystem INSTANCE = new DriveSubsystem();

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0), // Front Left
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0), // Front Right
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0), // Rear Left
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0) // Rear Right
    );

    private final SwerveDriveOdometry odometry;

    public final AHRS navX = new AHRS(SPI.Port.kMXP); // NavX Sensor

    private final SwerveModule[] swerveModules = new SwerveModule[]{
            new SwerveModule(Constants.CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID, Constants.CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID, Constants.CHASSIS_FRONT_LEFT_CANCODER_ID, Constants.CHASSIS_FRONT_LEFT_ANGLE_OFFSET_DEGREES), // Front Left
            new SwerveModule(Constants.CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID, Constants.CHASSIS_FRONT_RIGHT_CANCODER_ID, Constants.CHASSIS_FRONT_RIGHT_ANGLE_OFFSET_DEGREES), // Front Right
            new SwerveModule(Constants.CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID, Constants.CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID, Constants.CHASSIS_BACK_LEFT_CANCODER_ID, Constants.CHASSIS_BACK_LEFT_ANGLE_OFFSET_DEGREES), // Rear Left
            new SwerveModule(Constants.CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID, Constants.CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID, Constants.CHASSIS_BACK_RIGHT_CANCODER_ID, Constants.CHASSIS_BACK_RIGHT_ANGLE_OFFSET_DEGREES), // Rear Right
    };

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        navX.zeroYaw();
        odometry = new SwerveDriveOdometry(
                kinematics, navX.getRotation2d(),
                getModulePositions(), Constants.CHASSIS_INITIAL_POSITION);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(navX.getRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(swerveModuleStates);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void syncPositions(){
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.syncPosition();
        }
    }

    public void stopAll(){
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.stop();
        }
    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Angle", odometry.getPoseMeters().getRotation().getDegrees());
        for (int i = 0; i < 4; i++) {
            SwerveModule mod = swerveModules[i];
            SmartDashboard.putNumber("angle offset" + i , mod.getCanCoder().getAbsolutePosition().getValue());
        }
    }

    public static class SwerveModule {
        private final TalonFX driveMotor;
        private final CANSparkMax angleMotor;
        private final CANcoder canCoder;
        private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                Constants.CHASSIS_DRIVE_MOTOR_KS,
                Constants.CHASSIS_DRIVE_MOTOR_KV,
                Constants.CHASSIS_DRIVE_MOTOR_KA
        );

        private Rotation2d lastangle;

        public SwerveModule(int driveMotorCANID, int angleMotorCANID, int canCoderCANID, double angleOffset) {
            driveMotor = new TalonFX(driveMotorCANID);
            angleMotor = new CANSparkMax(angleMotorCANID, CANSparkMax.MotorType.kBrushless);
            canCoder = new CANcoder(canCoderCANID);


            // factory reset drive and angle motors
            driveMotor.getConfigurator().apply(
                    new Slot0Configs().withKP(Constants.CHASSIS_DRIVE_MOTOR_KP).withKD(Constants.CHASSIS_DRIVE_MOTOR_KD)
            );
            angleMotor.restoreFactoryDefaults();
            // factory reset canCoder
            canCoder.getConfigurator().apply(
                    new MagnetSensorConfigs()
                            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                            .withMagnetOffset(angleOffset)
            );
            lastangle = getState().angle;

            angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            double angleRatio = 360 / Constants.CHASSIS_ANGLE_GEAR_RATIO;
            angleMotor.getPIDController().setP(Constants.CHASSIS_ANGLE_MOTOR_KP / angleRatio);
            angleMotor.getPIDController().setD(Constants.CHASSIS_ANGLE_MOTOR_KD / angleRatio);
            angleMotor.getPIDController().setFF(Constants.CHASSIS_ANGLE_MOTOR_KF / angleRatio);
            angleMotor.getPIDController().setSmartMotionMaxVelocity(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY * angleRatio, 0);
            angleMotor.getPIDController().setSmartMotionMaxAccel(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION * angleRatio, 0);
            angleMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR * angleRatio, 0);
            // set gear ratio conversion to spark max
            angleMotor.getEncoder().setPositionConversionFactor(angleRatio);
            angleMotor.getEncoder().setVelocityConversionFactor(angleRatio);
            // sync canCoder's position to angleMotor's position
            syncPosition();
        }

        public void syncPosition() {
            angleMotor.getEncoder().setPosition(-canCoder.getAbsolutePosition().getValue()*360);
        }

        public final double getDriveGearRatio() {
            return switch (Constants.CHASSIS_DRIVE_GEAR_RATIO) {
                case L1 -> 8.14;
                case L2 -> 6.75;
                case L3 -> 6.12;
                case L4 -> 5.14;
            };
        }

        public void setDesiredState(SwerveModuleState state) {
            state = optimize(state, getRotation());
            driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond * getDriveGearRatio() / Constants.CHASSIS_WHEEL_DIAMETER_METERS / Math.PI).withFeedForward(feedforward.calculate(state.speedMetersPerSecond)));
            angleMotor.getPIDController().setReference(-state.angle.getDegrees(), CANSparkMax.ControlType.kSmartMotion);
        }

        public void setAngle(SwerveModuleState state) {
            Rotation2d angle = (Math.abs(state.speedMetersPerSecond)<= (Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND*0.01)) ? lastangle : state.angle;
            angleMotor.getPIDController().setReference(angle.getDegrees(), CANSparkBase.ControlType.kSmartMotion);
            lastangle = angle;
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    driveMotor.getVelocity().getValue() / getDriveGearRatio() * Math.PI * Constants.CHASSIS_WHEEL_DIAMETER_METERS,
                    getRotation()
            );
        }

        /**
         * Minimize the change in heading the desired swerve module state would require by potentially
         * reversing the direction the wheel spins. Customized from WPILib's version to include placing
         * in appropriate scope for CTRE and REV onboard control.
         *
         * @param desiredState The desired state.
         * @param currentAngle The current module angle.
         */
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90){
                targetSpeed = -targetSpeed;
                if (delta > 90) {
                    targetAngle -= 180;
                } else {
                    targetAngle += 180;
                }
            }
            return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
        }

        /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = scopeReference % 360;
            if (lowerOffset >= 0) {
                lowerBound = scopeReference - lowerOffset;
                upperBound = scopeReference + (360 - lowerOffset);
            } else {
                upperBound = scopeReference - lowerOffset;
                lowerBound = scopeReference - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle += 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - scopeReference > 180) {
                newAngle -= 360;
            } else if (newAngle - scopeReference < -180) {
                newAngle += 360;
            }
            return newAngle;
        }

        public final SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    driveMotor.getPosition().getValue()/ getDriveGearRatio() * Math.PI * Constants.CHASSIS_WHEEL_DIAMETER_METERS,
                    getRotation());
        }

        private Rotation2d getRotation(){
            return Rotation2d.fromDegrees(-angleMotor.getEncoder().getPosition());
        }

        public void stop() {
            driveMotor.stopMotor();
            angleMotor.stopMotor();
        }

        public CANcoder getCanCoder() {
            return canCoder;
        }
    }

    public enum GearRatio {
        L1,
        L2,
        L3,
        L4
    }
}

