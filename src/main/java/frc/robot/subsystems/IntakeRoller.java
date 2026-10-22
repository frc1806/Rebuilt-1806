package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Constants.CollectorConstants;



public class IntakeRoller extends SubsystemBase {



private TalonFX mCollectorMotor = new TalonFX(RobotMap.INTAKE);
private TalonFX mCollectorMotorTwo = new TalonFX(RobotMap.INTAKE_TWO);

private VoltageOut mCollectorVoltageOutStop = new VoltageOut(0.0);
private VoltageOut mCollectorVoltageOutGo = new VoltageOut(12.0);


//private TalonFX mSliderMotor = new TalonFX(RobotMap.SLIDER);
//private TalonFXConfiguration mSliderConfig = new TalonFXConfiguration();
//private SoftwareLimitSwitchConfigs mSliderSoftLimitConfig = new SoftwareLimitSwitchConfigs();

private NetworkTable mRobotTable = NetworkTableInstance.getDefault().getTable("Robot");


private static IntakeRoller S_COLLECTOR = new IntakeRoller();
public static IntakeRoller GetInstance(){
    return S_COLLECTOR;
}

enum SliderStates{
    kIdle,
    kExtending,
    kExtended,
    kRetracting,
    kRetracted,
    kZeroExtending,
    kShake
}

    private SliderStates mSliderState = SliderStates.kIdle;


private IntakeRoller(){

    /*TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.CollectorConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMIT)).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    mCollectorMotor.getConfigurator().apply(rollerConfig);*/

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.smartCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMIT);
    intakeConfig.inverted(false);
    intakeConfig.idleMode(IdleMode.kCoast);

    ClosedLoopConfig intakeRollerSpeedPIDConfig = new ClosedLoopConfig();
    intakeRollerSpeedPIDConfig.p(Constants.CollectorConstants.INTAKE_SPEED_KP)
        .i(Constants.CollectorConstants.INTAKE_SPEED_KI)
        .d(Constants.CollectorConstants.INTAKE_SPEED_KD).minOutput(0.0)
        .feedForward
            .kS(Constants.CollectorConstants.INTAKE_SPEED_KS)
            .kV(Constants.CollectorConstants.INTAKE_SPEED_KV);
    intakeConfig.apply(intakeRollerSpeedPIDConfig);
    
  TalonFXConfiguration collectorMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs collectorCurrentLimitConfig = new CurrentLimitsConfigs();
    collectorCurrentLimitConfig.StatorCurrentLimit = Constants.CollectorConstants.STATOR_CURRENT_LIMIT;
    collectorCurrentLimitConfig.SupplyCurrentLimit = Constants.CollectorConstants.COLLECTOR_MOTOR_CURRENT_LIMIT;
    MotorOutputConfigs collectorOutputConfig = new MotorOutputConfigs();
    collectorOutputConfig.Inverted= InvertedValue.CounterClockwise_Positive;
    collectorOutputConfig.NeutralMode = NeutralModeValue.Coast;
    collectorMotorConfig.withMotorOutput(collectorOutputConfig);
    mCollectorMotor.getConfigurator().apply(collectorMotorConfig);

    collectorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
    collectorMotorConfig.withMotorOutput(collectorOutputConfig);
    mCollectorMotorTwo.getConfigurator().apply(collectorMotorConfig);

    mCollectorMotorTwo.setControl(new Follower(mCollectorMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    /*TalonFXConfiguration sliderMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs sliderCurrentLimitConfig = new CurrentLimitsConfigs();
    sliderCurrentLimitConfig.StatorCurrentLimit = 120;
    sliderCurrentLimitConfig.SupplyCurrentLimit = Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT;*/
    //mSliderConfig.smartCurrentLimit(Constants.CollectorConstants.SLIDER_MOTOR_CURRENT_LIMIT);
    //mSliderConfig.voltageCompensation(Constants.CollectorConstants.SLIDER_MOTOR_VOLTAGE_COMPENSATION);
    //MotorOutputConfigs sliderOutputConfig = new MotorOutputConfigs();
    //sliderOutputConfig.Inverted= InvertedValue.Clockwise_Positive;
    //sliderOutputConfig.NeutralMode = NeutralModeValue.Coast;
    //mSliderConfig.inverted(Constants.CollectorConstants.SLIDER_MOTOR_INVERTED);
    /*sliderMotorConfig.withCurrentLimits(sliderCurrentLimitConfig).withMotorOutput(sliderOutputConfig);
    sliderMotorConfig.Slot0.kP = Constants.CollectorConstants.SLIDER_MOTOR_KP;
    sliderMotorConfig.Slot0.kI = Constants.CollectorConstants.SLIDER_MOTOR_KI;
    sliderMotorConfig.Slot0.kD = Constants.CollectorConstants.SLIDER_MOTOR_KD;

    sliderMotorConfig.Slot1.kP = Constants.CollectorConstants.SLIDER_MOTOR_SOFT_KP;
    sliderMotorConfig.Slot1.kI = Constants.CollectorConstants.SLIDER_MOTOR_SOFT_KI;
    sliderMotorConfig.Slot1.kD = Constants.CollectorConstants.SLIDER_MOTOR_SOFT_KD;
    */
    //mSliderSoftLimitConfig.ForwardSoftLimitThreshold = Constants.CollectorConstants.OUT_POSITION;
    //mSliderSoftLimitConfig.ForwardSoftLimitEnable = true;
    //mSliderSoftLimitConfig.ReverseSoftLimitEnable = true;
    //mSliderSoftLimitConfig.ReverseSoftLimitThreshold = Constants.CollectorConstants.IN_POSITION;
    //sliderMotorConfig.Feedback.SensorToMechanismRatio = 1;




    //mSliderMotor.getConfigurator().apply(sliderMotorConfig);


}



public void stop(){
    mCollectorMotor.stopMotor();
    mSliderState = SliderStates.kIdle;
} 

public void stopIntake(){
      if(RobotContainer.launcher.isLaunching()){
       mCollectorMotor.setControl(mCollectorVoltageOutGo);       
     }
     else{
         mCollectorMotor.stopMotor();
    }
}

public void intake(){
       // mCollectorMotor.setVoltage(6.0 + (Math.abs(RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond) * (6.0/5.0)));
        mCollectorMotor.setControl(mCollectorVoltageOutGo);   
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

        /*mRobotTable.putValue("Collector/Position", NetworkTableValue.makeDouble(mSliderMotor.getPosition().getValue().in(Rotations)));
        mRobotTable.putValue("Collector/CollectorCurrent", NetworkTableValue.makeDouble(mCollectorMotor.getSupplyCurrent().getValueAsDouble()));
        mRobotTable.putValue("Collector/CollectorState", NetworkTableValue.makeString(mSliderState.name()));
    }
*/
    /*public void extend (){
        mSliderState = SliderStates.kExtending;
    }

    public void retract (){
        mSliderState = SliderStates.kRetracting;
    }

    public void shake (){
        mSliderState = SliderStates.kShake;
    }*/

    /*public void setCoastMode(){
        mSliderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mSliderMotor.getConfigurator().apply(mSliderConfig);
    }*/

    /*public void setBreakMode(){
        mSliderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSliderMotor.getConfigurator().apply(mSliderConfig);
        
    }*/

    //public Trigger isIntakeExdendedButIn(){
      //  return new Trigger(() ->  mSliderState == SliderStates.kExtended && mSliderMotor.getPosition().getValue().in(Rotations) < Constants.CollectorConstants.OUT_POSITION - 3.0);
    }

   /* public void zeroExtending (){
        mSliderSoftLimitConfig.forwardSoftLimitEnabled(false);
        mSliderSoftLimitConfig.reverseSoftLimitEnabled(false);
        mSliderConfig.apply(mSliderSoftLimitConfig);
        mSliderMotor.configure(mSliderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mSliderState = SliderStates.kZeroExtending;
    }*/
}
