package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
    CANSparkMax hangerMotor1 = new CANSparkMax(Constants.HANGER_MOTOR_1_ID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax hangerMotor2 = new CANSparkMax(Constants.HANGER_MOTOR_2_ID, CANSparkMax.MotorType.kBrushless);
    private boolean zeroing = false;
    private double zeroStartTime = 0;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this HangSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static HangerSubsystem INSTANCE = new HangerSubsystem();

    /**
     * Returns the Singleton instance of this HangSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code HangSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static HangerSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this HangSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private HangerSubsystem() {
        // configure motors
        hangerMotor1.restoreFactoryDefaults();
        hangerMotor2.restoreFactoryDefaults();
        hangerMotor1.setInverted(Constants.HANGER_MOTOR_1_INVERTED);
        hangerMotor2.setInverted(Constants.HANGER_MOTOR_2_INVERTED);
        hangerMotor1.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT);
        hangerMotor2.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT);
        // set pid constants
        hangerMotor1.getPIDController().setP(Constants.HANGER_MOTOR_KP);
        hangerMotor1.getPIDController().setD(Constants.HANGER_MOTOR_KD);
        hangerMotor1.getPIDController().setFF(Constants.HANGER_MOTOR_KF);
        hangerMotor1.getPIDController().setSmartMotionMaxVelocity(Constants.HANGER_MOTOR_SMART_MOTION_MAX_VELOCITY, 0);
        hangerMotor1.getPIDController().setSmartMotionMaxAccel(Constants.HANGER_MOTOR_SMART_MOTION_MAX_ACCELERATION, 0);
        hangerMotor1.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.HANGER_MOTOR_SMART_MOTION_ALLOWABLE_ERROR, 0);
        hangerMotor2.getPIDController().setP(Constants.HANGER_MOTOR_KP);
        hangerMotor2.getPIDController().setD(Constants.HANGER_MOTOR_KD);
        hangerMotor2.getPIDController().setFF(Constants.HANGER_MOTOR_KF);
        hangerMotor2.getPIDController().setSmartMotionMaxVelocity(Constants.HANGER_MOTOR_SMART_MOTION_MAX_VELOCITY, 0);
        hangerMotor2.getPIDController().setSmartMotionMaxAccel(Constants.HANGER_MOTOR_SMART_MOTION_MAX_ACCELERATION, 0);
        hangerMotor2.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.HANGER_MOTOR_SMART_MOTION_ALLOWABLE_ERROR, 0);
        // set gear ratio
        hangerMotor1.getEncoder().setPositionConversionFactor(1/Constants.HANGER_MOTOR_GEAR_RATIO);
        hangerMotor1.getEncoder().setVelocityConversionFactor(1/Constants.HANGER_MOTOR_GEAR_RATIO);
        hangerMotor2.getEncoder().setPositionConversionFactor(1/Constants.HANGER_MOTOR_GEAR_RATIO);
        hangerMotor2.getEncoder().setVelocityConversionFactor(1/Constants.HANGER_MOTOR_GEAR_RATIO);
        hangerMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        hangerMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hangerMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        hangerMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hangerMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        hangerMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if(zeroing && System.currentTimeMillis() - zeroStartTime >= 1000){
            if(hangerMotor1.getEncoder().getVelocity() < 0.001) {
                hangerMotor1.set(0);
                hangerMotor1.getEncoder().setPosition(0);
                hangerMotor1.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT);
            }
            if(hangerMotor2.getEncoder().getVelocity() < 0.001) {
                hangerMotor2.set(0);
                hangerMotor2.getEncoder().setPosition(0);
                hangerMotor2.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT);
            }
            if (hangerMotor1.getEncoder().getVelocity() < 0.001 && hangerMotor2.getEncoder().getVelocity() < 0.001) {
                zeroing = false;
            }
        }
    }

    public void zeroHanger() {
//        zeroing = true;
//        zeroStartTime = System.currentTimeMillis();
//        hangerMotor1.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT_ZEROING);
//        hangerMotor1.set(-0.4);
//        hangerMotor2.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT_ZEROING);
//        hangerMotor2.set(-0.4);
        hangerMotor1.getEncoder().setPosition(0);
        hangerMotor2.getEncoder().setPosition(0);
    }

    public void setHangerPosition(double position) {
        if (zeroing) return;
        hangerMotor1.getPIDController().setReference(position, CANSparkBase.ControlType.kSmartMotion);
        hangerMotor2.getPIDController().setReference(position, CANSparkBase.ControlType.kSmartMotion);
    }

    public void setHangerSpeed(double speed) {
        if (zeroing) return;
        hangerMotor1.set(speed);
        hangerMotor2.set(speed);
    }

    // expand hanger
    public void expandHanger() {
        setHangerPosition(Constants.HANGER_MAX_LENGTH);
    }

    // retract hanger
    public void retractHanger() {
        setHangerPosition(0);
    }
}

