package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax angleMotor1 = new CANSparkMax(Constants.SHOOTER_ANGLE_MOTOR_1_ID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax angleMotor2 = new CANSparkMax(Constants.SHOOTER_ANGLE_MOTOR_2_ID, CANSparkMax.MotorType.kBrushless);
    TalonFX flywheelMotor = new TalonFX(Constants.SHOOTER_FLYWHEEL_MOTOR_ID);
    private boolean zeroing = false;
    private double zeroStartTime = 0;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ShootSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    /**
     * Returns the Singleton instance of this ShootSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShootSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ShootSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        // Set flywheel motor parameters
        flywheelMotor.getConfigurator().refresh(
                new TalonFXConfiguration().withSlot0(
                        new Slot0Configs().withKP(Constants.SHOOTER_FLYWHEEL_MOTOR_KP)
                                .withKD(Constants.SHOOTER_FLYWHEEL_MOTOR_KD)
                                .withKV(Constants.SHOOTER_FLYWHEEL_MOTOR_KF)
                ).withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Constants.SHOOTER_FLYWHEEL_MOTOR_CURRENT_LIMIT)
                )
        );
        flywheelMotor.setInverted(Constants.SHOOTER_FLYWHEEL_MOTOR_INVERTED);

        // Set angle motor parameters
        angleMotor1.restoreFactoryDefaults();
        angleMotor1.setInverted(Constants.SHOOTER_ANGLE_MOTOR_1_INVERTED);
        SparkPIDController anglePID1 = angleMotor1.getPIDController();
        anglePID1.setP(Constants.SHOOTER_ANGLE_MOTOR_KP);
        anglePID1.setD(Constants.SHOOTER_ANGLE_MOTOR_KD);
        anglePID1.setFF(Constants.SHOOTER_ANGLE_MOTOR_KF);
        anglePID1.setSmartMotionMaxVelocity(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY, 0);
        anglePID1.setSmartMotionMaxAccel(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION, 0);
        anglePID1.setSmartMotionAllowedClosedLoopError(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR, 0);
        angleMotor1.getEncoder().setPositionConversionFactor(1/Constants.SHOOTER_ANGLE_GEAR_RATIO);
        angleMotor1.getEncoder().setVelocityConversionFactor(1/Constants.SHOOTER_ANGLE_GEAR_RATIO);
        // set current limits
        angleMotor1.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
        angleMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SHOOTER_ANGLE_MAX);
        angleMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        angleMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        angleMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        angleMotor2.restoreFactoryDefaults();
        angleMotor2.setInverted(Constants.SHOOTER_ANGLE_MOTOR_2_INVERTED);
        SparkPIDController anglePID2 = angleMotor2.getPIDController();
        anglePID2.setP(Constants.SHOOTER_ANGLE_MOTOR_KP);
        anglePID2.setD(Constants.SHOOTER_ANGLE_MOTOR_KD);
        anglePID2.setFF(Constants.SHOOTER_ANGLE_MOTOR_KF);
        anglePID2.setSmartMotionMaxVelocity(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY, 0);
        anglePID2.setSmartMotionMaxAccel(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION, 0);
        anglePID2.setSmartMotionAllowedClosedLoopError(Constants.SHOOTER_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR, 0);
        angleMotor2.getEncoder().setPositionConversionFactor(1/Constants.SHOOTER_ANGLE_GEAR_RATIO);
        angleMotor2.getEncoder().setVelocityConversionFactor(1/Constants.SHOOTER_ANGLE_GEAR_RATIO);
        // set current limits
        angleMotor2.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
        angleMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SHOOTER_ANGLE_MAX);
        angleMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        angleMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        angleMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    @Override
    public void periodic() {
        if(zeroing && System.currentTimeMillis() - zeroStartTime >= 3000){
            zeroing = false;
            angleMotor1.set(0);
            angleMotor1.getEncoder().setPosition(0);
            angleMotor1.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
            angleMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
            angleMotor2.set(0);
            angleMotor2.getEncoder().setPosition(0);
            angleMotor2.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
            angleMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        }
        if(zeroing && System.currentTimeMillis() - zeroStartTime >= 1000){
            if(angleMotor1.getEncoder().getVelocity() < 0.3) {
                angleMotor1.set(0);
                angleMotor1.getEncoder().setPosition(0);
                angleMotor1.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
                angleMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
            }
            if(angleMotor2.getEncoder().getVelocity() < 0.3) {
                angleMotor2.set(0);
                angleMotor2.getEncoder().setPosition(0);
                angleMotor2.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT);
                angleMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
            }
            if (angleMotor1.getEncoder().getVelocity() < 0.3 && angleMotor2.getEncoder().getVelocity() < 0.3) {
                zeroing = false;
            }
        }
    }

    public void zeroAngle() {
        zeroing = true;
        zeroStartTime = System.currentTimeMillis();
        angleMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        angleMotor1.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT_ZEROING);
        angleMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        angleMotor2.setSmartCurrentLimit(Constants.SHOOTER_ANGLE_MOTOR_CURRENT_LIMIT_ZEROING);
        angleMotor1.set(-0.3);
        angleMotor2.set(-0.3);
    }

    public void setAngle(double angle) {
        if (zeroing) return;
        angleMotor1.getPIDController().setReference(angle, CANSparkMax.ControlType.kSmartMotion);
        angleMotor2.getPIDController().setReference(angle, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setFlywheel(double rpm) {
        flywheelMotor.setControl(new VelocityDutyCycle(rpm/60));
    }

    public void stopMotors() {
        angleMotor1.set(0);
        flywheelMotor.set(0);
    }
}

