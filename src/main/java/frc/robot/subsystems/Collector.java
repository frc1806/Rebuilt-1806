package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Constants.CollectorConstants;



public class Collector extends SubsystemBase {



private SparkFlex mCollectorMotor = new SparkFlex(RobotMap.INTAKE, MotorType.kBrushless);
private SparkFlex mCollectorMotorTwo = new SparkFlex(RobotMap.INTAKE_TWO, MotorType.kBrushless);


private TalonFX mSliderMotor = new TalonFX(RobotMap.SLIDER);
private TalonFXConfiguration mSliderConfig = new TalonFXConfiguration();
private SoftwareLimitSwitchConfigs mSliderSoftLimitConfig = new SoftwareLimitSwitchConfigs();

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
    /*TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.CollectorConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMIT)).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    mCollectorMotor.getConfigurator().apply(rollerConfig);*/

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.smartCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMIT);
    intakeConfig.inverted(false);

    ClosedLoopConfig intakeRollerSpeedPIDConfig = new ClosedLoopConfig();
    intakeRollerSpeedPIDConfig.p(Constants.CollectorConstants.INTAKE_SPEED_KP)
        .i(Constants.CollectorConstants.INTAKE_SPEED_KI)
        .d(Constants.CollectorConstants.INTAKE_SPEED_KD)
        .feedForward
            .kS(Constants.CollectorConstants.INTAKE_SPEED_KS)
            .kV(Constants.CollectorConstants.INTAKE_SPEED_KV);
    intakeConfig.apply(intakeRollerSpeedPIDConfig);
    

    mCollectorMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeConfig.follow(mCollectorMotor.getDeviceId(), true);
    mCollectorMotorTwo.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfiguration sliderMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs sliderCurrentLimitConfig = new CurrentLimitsConfigs();
    sliderCurrentLimitConfig.StatorCurrentLimit = 120;
    sliderCurrentLimitConfig.SupplyCurrentLimit = Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT;
    //mSliderConfig.smartCurrentLimit(Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT);
    //mSliderConfig.voltageCompensation(Constants.CollectorConstants.SLIDER_MOTOR_VOLTAGE_COMPENSATION);
    MotorOutputConfigs sliderOutputConfig = new MotorOutputConfigs();
    sliderOutputConfig.Inverted= InvertedValue.Clockwise_Positive;
    sliderOutputConfig.NeutralMode = NeutralModeValue.Brake;
    //mSliderConfig.inverted(Constants.CollectorConstants.SLIDER_MOTOR_INVERTED);
    sliderMotorConfig.withCurrentLimits(sliderCurrentLimitConfig).withMotorOutput(sliderOutputConfig);
    sliderMotorConfig.Slot0.kP = Constants.CollectorConstants.SLIDER_MOTOR_KP;
    sliderMotorConfig.Slot0.kI = Constants.CollectorConstants.SLIDER_MOTOR_KI;
    sliderMotorConfig.Slot0.kD = Constants.CollectorConstants.SLIDER_MOTOR_KD;

    mSliderSoftLimitConfig.ForwardSoftLimitThreshold = Constants.CollectorConstants.OUT_POSITION;
    mSliderSoftLimitConfig.ForwardSoftLimitEnable = true;
    mSliderSoftLimitConfig.ReverseSoftLimitEnable = true;
    mSliderSoftLimitConfig.ReverseSoftLimitThreshold = Constants.CollectorConstants.IN_POSITION;
    sliderMotorConfig.Feedback.SensorToMechanismRatio = 1;




    mSliderMotor.getConfigurator().apply(sliderMotorConfig);


}



public void stop(){
    mCollectorMotor.stopMotor();
    mSliderState = SliderStates.kIdle;
} 

public void stopIntake(){
    if(mSliderMotor.getPosition().getValue().in(Rotations) > CollectorConstants.SLIDER_SAFE_EXTENSION_MIN){
        if(RobotContainer.launcher.isLaunching()){
            mCollectorMotor.setVoltage(6.0);
        }
        else{
            mCollectorMotor.stopMotor();
        }
    }
    else{
        mCollectorMotor.stopMotor();
    }

}

public void intake(){
    if(mSliderMotor.getPosition().getValue().in(Rotations) > CollectorConstants.SLIDER_SAFE_EXTENSION_MIN){
        mCollectorMotor.setVoltage(6.0 + (Math.abs(RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond) * (6.0/5.0)));
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
            if(Math.abs(mSliderMotor.getPosition().getValue().in(Rotations) - Constants.CollectorConstants.OUT_POSITION) < 1.0){
                mSliderMotor.set(0.0);
            }
            else{
                mSliderMotor.setControl(new PositionVoltage(Constants.CollectorConstants.OUT_POSITION));

            }
                //mSliderMotor.getClosedLoopController().setSetpoint(Math.min(Constants.CollectorConstants.OUT_POSITION, Math.max( 0.0, (Constants.CollectorConstants.OUT_POSITION - (16.0* RobotContainer.launcher.getLaunchingTime() + (6 * Math.sin(4 * Math.PI * RobotContainer.launcher.getLaunchingTime())))))), ControlType.kPosition);
                //mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.OUT_POSITION, ControlType.kPosition);
                break;

            default:
            case kIdle:
                mSliderMotor.set(0);
                break;

            case kRetracting:
                if(Math.abs(mSliderMotor.getPosition().getValue().in(Rotations) - Constants.CollectorConstants.IN_POSITION) < 1.0){
                    mSliderMotor.set(0.0);
                }
                else{
                    mSliderMotor.setControl(new PositionVoltage(Constants.CollectorConstants.IN_POSITION));
                }

                //mSliderMotor.getClosedLoopController().setSetpoint(Constants.CollectorConstants.IN_POSITION, ControlType.kPosition);
                break;

            case kShake:
                mSliderMotor.set(Constants.CollectorConstants.SLIDER_SHAKE_POWER_FACTOR * Math.sin(Timer.getFPGATimestamp()* Constants.CollectorConstants.SLIDER_SHAKE_TIME_SCALER));
                mCollectorMotor.set(.8);
                break;

            /*case kZeroExtending:
                mSliderMotor.set(.3);
                if(Math.abs(mSliderMotor.getStatorCurrent()) > Constants.CollectorConstants.INTAKE_ZEROING_CURRENT_THRESHOLD && Math.abs(mSliderMotor.getPosition().getValue().in(Rotations)) < Constants.CollectorConstants.INTAKE_ZEROING_VELOCITY_THRESHOLD){
                    mSliderMotor.().setPosition(0);
                    mSliderState = SliderStates.kIdle;
                    mSliderMotor.set(0);
                    mSliderSoftLimitConfig.forwardSoftLimitEnabled(true);
                    mSliderSoftLimitConfig.reverseSoftLimitEnabled(true);
                    mSliderConfig.apply(mSliderSoftLimitConfig);
                    mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                }
                mSliderMotor.getEncoder().getVelocity();
                break;
                */
        }

        mRobotTable.putValue("Collector/Position", NetworkTableValue.makeDouble(mSliderMotor.getPosition().getValue().in(Rotations)));
        mRobotTable.putValue("Collector/CollectorCurrent", NetworkTableValue.makeDouble(mCollectorMotor.getEncoder().getVelocity()));
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
        mSliderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mSliderMotor.getConfigurator().apply(mSliderConfig);
    }

    public void setBreakMode(){
        mSliderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSliderMotor.getConfigurator().apply(mSliderConfig);
        
    }

   /* public void zeroExtending (){
        mSliderSoftLimitConfig.forwardSoftLimitEnabled(false);
        mSliderSoftLimitConfig.reverseSoftLimitEnabled(false);
        mSliderConfig.apply(mSliderSoftLimitConfig);
        mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mSliderState = SliderStates.kZeroExtending;
    }*/
}
