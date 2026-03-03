package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.LauncherSubSystem.LauncherStates;

public class Collector extends SubsystemBase {

private TalonFX mCollectorMotor = new TalonFX(RobotMap.INTAKE);

private SparkFlex mSliderMotor = new SparkFlex(RobotMap.SLIDER, MotorType.kBrushless);
private SparkFlexConfig mSliderConfig = new SparkFlexConfig();
private SoftLimitConfig mSliderSoftLimitConfig = new SoftLimitConfig();

private NetworkTable mRobotTable = NetworkTableInstance.getDefault().getTable("Robot");

private static Collector S_COLLECTOR = new Collector();
public static Collector GetInstance(){
    return S_COLLECTOR;
}

enum SliderStates{
    kIdle,
    kExtending,
    kRetracting,
    kZeroExtending,
    kShake
}

    private SliderStates mSliderState = SliderStates.kIdle;


private Collector(){
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.CollectorConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMIT)).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    mCollectorMotor.getConfigurator().apply(rollerConfig);

    mSliderConfig.smartCurrentLimit(Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT);
    mSliderConfig.voltageCompensation(Constants.CollectorConstants.SLIDER_MOTOR_VOLTAGE_COMPENSATION);
    mSliderConfig.inverted(Constants.CollectorConstants.SLIDER_MOTOR_INVERTED);
    mSliderSoftLimitConfig.forwardSoftLimit(Constants.CollectorConstants.OUT_POSITION);
    mSliderSoftLimitConfig.forwardSoftLimitEnabled(false);
    mSliderSoftLimitConfig.reverseSoftLimitEnabled(false);
    mSliderSoftLimitConfig.reverseSoftLimit(Constants.CollectorConstants.IN_POSITION);
    mSliderConfig.apply(mSliderSoftLimitConfig);
    mSliderConfig.idleMode(IdleMode.kBrake);


    mSliderConfig.encoder.positionConversionFactor(1);
    mSliderConfig.encoder.velocityConversionFactor(1);
    mSliderConfig.closedLoop.pid(Constants.CollectorConstants.SLIDER_MOTOR_KP, Constants.CollectorConstants.SLIDER_MOTOR_KI, Constants.CollectorConstants.SLIDER_MOTOR_KD);
    mSliderConfig.closedLoop.maxMotion.cruiseVelocity(50.0);
    mSliderConfig.closedLoop.maxMotion.maxAcceleration(10.0);


    mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    mSliderConfig.closedLoop.pid(Constants.CollectorConstants.SLIDER_MOTOR_KP, Constants.CollectorConstants.SLIDER_MOTOR_KI, Constants.CollectorConstants.SLIDER_MOTOR_KD);


}



public void stop(){
    mCollectorMotor.stopMotor();
    mSliderState = SliderStates.kIdle;
} 

public void intake(){
    if(mSliderMotor.getEncoder().getPosition() > CollectorConstants.SLIDER_SAFE_EXTENSION_MIN){
        mCollectorMotor.setVoltage(12.0);
    }
    else{
        mCollectorMotor.setVoltage(0);
    }
    
}

public void clean(){
    mCollectorMotor.setVoltage(3.0);
}

public void outtake(){
    mCollectorMotor.setVoltage(-12.0);
}

    @SuppressWarnings("unchecked")
    @Override
    public void periodic() {
        switch (mSliderState) {
            case kExtending:
                mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.OUT_POSITION, ControlType.kPosition);
                break;

            default:
            case kIdle:
                mSliderMotor.set(0);
                break;

            case kRetracting:
                mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.IN_POSITION, ControlType.kPosition);
                break;

            case kShake:
                mSliderMotor.set(Constants.CollectorConstants.SLIDER_SHAKE_POWER_FACTOR * Math.sin(Timer.getFPGATimestamp()* Constants.CollectorConstants.SLIDER_SHAKE_TIME_SCALER));
                mCollectorMotor.set(.8);
                break;

            case kZeroExtending:
                mSliderMotor.set(.3);
                if(Math.abs(mSliderMotor.getOutputCurrent()) > Constants.CollectorConstants.INTAKE_ZEROING_CURRENT_THRESHOLD && Math.abs(mSliderMotor.getEncoder().getVelocity()) < Constants.CollectorConstants.INTAKE_ZEROING_VELOCITY_THRESHOLD){
                    mSliderMotor.getEncoder().setPosition(0);
                    mSliderState = SliderStates.kIdle;
                    mSliderMotor.set(0);
                    mSliderSoftLimitConfig.forwardSoftLimitEnabled(true);
                    mSliderSoftLimitConfig.reverseSoftLimitEnabled(true);
                    mSliderConfig.apply(mSliderSoftLimitConfig);
                    mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                }
                mSliderMotor.getEncoder().getVelocity();
                break;
        }

        mRobotTable.putValue("Collector/Position", NetworkTableValue.makeDouble(mSliderMotor.getEncoder().getPosition()));
        mRobotTable.putValue("Collector/CollectorCurrent", NetworkTableValue.makeDouble(mSliderMotor.getOutputCurrent()));
        mRobotTable.putValue("Collector/CollectorState", NetworkTableValue.makeString(mSliderState.name()));
    }

    public void extend (){
        mSliderState = SliderStates.kExtending;
    }

    public void retract (){
        mSliderState = SliderStates.kRetracting;
    }

    public void shake (){
        mSliderState = SliderStates.kShake;
    }

    public void setCoastMode(){
        mSliderConfig.idleMode(IdleMode.kCoast);
        mSliderMotor.configure(mSliderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setBreakMode(){
        mSliderConfig.idleMode(IdleMode.kCoast);
        mSliderMotor.configure(mSliderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
    }

    public void zeroExtending (){
        mSliderSoftLimitConfig.forwardSoftLimitEnabled(false);
        mSliderSoftLimitConfig.reverseSoftLimitEnabled(false);
        mSliderConfig.apply(mSliderSoftLimitConfig);
        mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mSliderState = SliderStates.kZeroExtending;
    }
}
