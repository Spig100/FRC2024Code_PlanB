package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    CANSparkMax intakeMotor = new CANSparkMax(Constants.COLLECTOR_INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax transferMotor = new CANSparkMax(Constants.COLLECTOR_TRANSFER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax sliderMotor = new CANSparkMax(Constants.COLLECTOR_SLIDER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private boolean zeroing = false;
    private double zeroStartTime = 0;
    private boolean enableIntake = false;
    private boolean shooting = false;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this CollectSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static CollectSubsystem INSTANCE = new CollectSubsystem();

    /**
     * Returns the Singleton instance of this CollectSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code CollectSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static CollectSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this CollectSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private CollectSubsystem() {
        // factory default motors
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(Constants.COLLECTOR_INTAKE_MOTOR_INVERTED);
        intakeMotor.setSmartCurrentLimit(Constants.COLLECTOR_INTAKE_MOTOR_CURRENT_LIMIT);
        intakeMotor.getPIDController().setP(Constants.COLLECTOR_INTAKE_MOTOR_KP);
        intakeMotor.getPIDController().setD(Constants.COLLECTOR_INTAKE_MOTOR_KD);
        intakeMotor.getPIDController().setFF(Constants.COLLECTOR_INTAKE_MOTOR_KF);
        intakeMotor.getEncoder().setPositionConversionFactor(1/Constants.COLLECTOR_INTAKE_MOTOR_GEAR_RATIO);
        intakeMotor.getEncoder().setVelocityConversionFactor(1/Constants.COLLECTOR_INTAKE_MOTOR_GEAR_RATIO);
        sliderMotor.restoreFactoryDefaults();
        sliderMotor.setInverted(Constants.COLLECTOR_SLIDER_MOTOR_INVERTED);
        sliderMotor.setSmartCurrentLimit(Constants.COLLECTOR_SLIDER_MOTOR_CURRENT_LIMIT);
        sliderMotor.getPIDController().setP(Constants.COLLECTOR_SLIDER_MOTOR_KP);
        sliderMotor.getPIDController().setD(Constants.COLLECTOR_SLIDER_MOTOR_KD);
        sliderMotor.getPIDController().setFF(Constants.COLLECTOR_SLIDER_MOTOR_KF);
        sliderMotor.getPIDController().setSmartMotionMaxVelocity(Constants.COLLECTOR_SLIDER_MOTOR_SMART_MOTION_MAX_VELOCITY,0);
        sliderMotor.getPIDController().setSmartMotionMaxAccel(Constants.COLLECTOR_SLIDER_MOTOR_SMART_MOTION_MAX_ACCELERATION,0);
        sliderMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.COLLECTOR_SLIDER_MOTOR_SMART_MOTION_ALLOWABLE_ERROR, 0);
        sliderMotor.getEncoder().setPositionConversionFactor(1/Constants.COLLECTOR_SLIDER_MOTOR_GEAR_RATIO);
        sliderMotor.getEncoder().setVelocityConversionFactor(1/Constants.COLLECTOR_SLIDER_MOTOR_GEAR_RATIO);
        transferMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(Constants.COLLECTOR_INTAKE_MOTOR_INVERTED);
        transferMotor.setInverted(Constants.COLLECTOR_TRANSFER_MOTOR_INVERTED);
        transferMotor.setSmartCurrentLimit(Constants.COLLECTOR_TRANSFER_MOTOR_CURRENT_LIMIT);
        transferMotor.getPIDController().setP(Constants.COLLECTOR_TRANSFER_MOTOR_KP);
        transferMotor.getPIDController().setD(Constants.COLLECTOR_TRANSFER_MOTOR_KD);
        transferMotor.getPIDController().setFF(Constants.COLLECTOR_TRANSFER_MOTOR_KF);
        transferMotor.getEncoder().setPositionConversionFactor(1/Constants.COLLECTOR_TRANSFER_MOTOR_GEAR_RATIO);
        transferMotor.getEncoder().setVelocityConversionFactor(1/Constants.COLLECTOR_TRANSFER_MOTOR_GEAR_RATIO);
        transferMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.setInverted(Constants.COLLECTOR_INTAKE_MOTOR_INVERTED);
    }

    @Override
    public void periodic() {
        if(zeroing && System.currentTimeMillis() - zeroStartTime >= 1000){
            if(sliderMotor.getEncoder().getVelocity() < 0.1) {
                zeroing = false;
                sliderMotor.set(0);
                sliderMotor.getEncoder().setPosition(0);
                sliderMotor.setSmartCurrentLimit(Constants.HANGER_MOTOR_CURRENT_LIMIT);
            }
        }else if(!zeroing){
            if(enableIntake){
                if (getIRSensor()) {
                    enableIntake = false;
                } else {
                    setIntakeMotor(450);
                    setTransferMotor(200);
                    expandSlider();
                }
            }else{
                setIntakeMotor(0);
                if (shooting){
                    setTransferMotor(450);
                }else{
                    setTransferMotor(0);
                }
                retractSlider();
            }
        }
    }

    public void zeroSlider() {
        zeroing = true;
        zeroStartTime = System.currentTimeMillis();
        sliderMotor.setSmartCurrentLimit(Constants.COLLECTOR_SLIDER_MOTOR_CURRENT_LIMIT_ZEROING);
        sliderMotor.set(-0.5);
    }

    public void setTransferMotor(double speed) {
        transferMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public void enableIntakeMotor() {
        enableIntake = true;
    }

    public void disableIntakeMotor() {
        enableIntake = false;
    }

    public void setSliderMotorPosition(double position) {
        if (zeroing) return;
        sliderMotor.getPIDController().setReference(position, CANSparkBase.ControlType.kSmartMotion);
    }

    public void expandSlider() {
        setSliderMotorPosition(Constants.SLIDER_MAXIMUM_LENGTH);
    }

    public void retractSlider() {
        setSliderMotorPosition(Constants.SLIDER_MINIMUM_LENGTH);
    }

    public boolean getIRSensor() {
        return intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }
}

