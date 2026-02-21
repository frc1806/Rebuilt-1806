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
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.LauncherSubSystem.LauncherStates;

public class Collector extends SubsystemBase {

private TalonFX mCollectorMotor = new TalonFX(RobotMap.INTAKE);

private SparkFlex mSliderMotor = new SparkFlex(RobotMap.SLIDER, MotorType.kBrushless);

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

    SparkFlexConfig sliderMotorConfig = new SparkFlexConfig();
    sliderMotorConfig.smartCurrentLimit(Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT);
    sliderMotorConfig.voltageCompensation(Constants.CollectorConstants.SLIDER_MOTOR_VOLTAGE_COMPENSATION);
    sliderMotorConfig.inverted(Constants.CollectorConstants.SLIDER_MOTOR_INVERTED);

    sliderMotorConfig.encoder.positionConversionFactor(1);
    sliderMotorConfig.encoder.velocityConversionFactor(1);


    mSliderMotor.configure(sliderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    sliderMotorConfig.closedLoop.pid(Constants.CollectorConstants.SLIDER_MOTOR_KP, Constants.CollectorConstants.SLIDER_MOTOR_KI, Constants.CollectorConstants.SLIDER_MOTOR_KD);


}



public void stop(){
    mCollectorMotor.stopMotor();
    mSliderState = SliderStates.kIdle;
} 

public void intake(){
    mCollectorMotor.setVoltage(12.0);
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
                mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.OUT_POSITION, ControlType.kMAXMotionPositionControl);
                break;

            default:
            case kIdle:
                mSliderMotor.set(0);
                break;

            case kRetracting:
                mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.IN_POSITION, ControlType.kMAXMotionPositionControl);
                break;

            case kShake:
                mSliderMotor.set(Constants.CollectorConstants.SLIDER_SHAKE_POWER_FACTOR * Math.sin(Timer.getFPGATimestamp()* Constants.CollectorConstants.SLIDER_SHAKE_TIME_SCALER));
                break;

            case kZeroExtending:
                mSliderMotor.set(.3);
                if(mSliderMotor.getOutputCurrent() > Constants.CollectorConstants.INTAKE_ZEROING_CURRENT_THRESHOLD && Math.abs(mSliderMotor.getEncoder().getVelocity()) < Constants.CollectorConstants.INTAKE_ZEROING_VELOCITY_THRESHOLD){
                    mSliderMotor.getEncoder().setPosition(0);
                    mSliderState = SliderStates.kIdle;
                    mSliderMotor.set(0);
                }
                mSliderMotor.getEncoder().getVelocity();
                break;
        }
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

    public void zeroExtending (){
        mSliderState = SliderStates.kZeroExtending;
    }
}
