package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeArmSubsystem extends SubsystemBase{

    private TalonFX mClimberFrontLead;// mClimberFrontFollower;//, mClimberRearLead; //mClimberRearFollower;

    public IntakeArmSubsystem(){
        mClimberFrontLead = new TalonFX(RobotMap.CLIMBER_1);
        mClimberFrontLead.setNeutralMode(NeutralModeValue.Brake);
        //mClimberFrontFollower = new TalonFX(RobotMap.CLIMBER_2);
        //mClimberRearLead = new TalonFX(RobotMap.CLIMBER_3);
        //mClimberRearFollower = new TalonFX(RobotMap.CLIMBER_4);

        CurrentLimitsConfigs climberCurrentLimitConfig = new CurrentLimitsConfigs();
        climberCurrentLimitConfig.StatorCurrentLimit = Constants.IntakeArmConstants.INTAKE_ARM_STATOR_CURRENT_LIMIT;
        climberCurrentLimitConfig.SupplyCurrentLimit = Constants.IntakeArmConstants.INTAKE_ARM_SUPPLY_CURRENT_LIMIT;

        //Follower frontFollower = new Follower(mClimberFrontLead.getDeviceID(), MotorAlignmentValue.Opposed);

        //Follower reaFollower = new Follower(mClimberRearLead.getDeviceID(), MotorAlignmentValue.Opposed);

        mClimberFrontLead.getConfigurator().apply(climberCurrentLimitConfig);
        //mClimberFrontFollower.getConfigurator().apply(climberCurrentLimitConfig);
       //mClimberRearLead.getConfigurator().apply(climberCurrentLimitConfig);
        //mClimberRearFollower.getConfigurator().apply(climberCurrentLimitConfig);
        
        MotorOutputConfigs leaderOutputConfigs = new MotorOutputConfigs();
        leaderOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        OpenLoopRampsConfigs mFrontLeadOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
        mFrontLeadOpenLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.25;

        mClimberFrontLead.getConfigurator().apply(leaderOutputConfigs);
        //mClimberRearLead.getConfigurator().apply(leaderOutputConfigs);

        Slot0Configs l1Slot0Configs = new Slot0Configs();
        l1Slot0Configs.kP = Constants.IntakeArmConstants.INTAKE_ARM_KP;
        l1Slot0Configs.kI = Constants.IntakeArmConstants.INTAKE_ARM_KI;
        l1Slot0Configs.kD = Constants.IntakeArmConstants.INTAKE_ARM_KD;
        mClimberFrontLead.getConfigurator().apply(l1Slot0Configs);

        //mClimberFrontFollower.setControl(frontFollower);
        //mClimberRearFollower.setControl(reaFollower);
    }

    @Override
    public void periodic(){
        NetworkTableInstance.getDefault().getTable("Robot/Climber").putValue("Intake Arm Angle", NetworkTableValue.makeDouble(getIntakeArmAngle()));
        
    }

    public void manualControl(double frontOutput){
        if(Math.abs(frontOutput) > 0.25){
            mClimberFrontLead.set(((Math.abs(frontOutput)-.25)) * (frontOutput <0?-1:1) * 0.5);
        }else{
            mClimberFrontLead.set(0.0);
        }

        //mClimberRearLead.set(rearOutput * 0.5);
    }

    public double getIntakeArmAngle(){
        return mClimberFrontLead.getPosition().getValueAsDouble() * Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT;
    }

    public void zeroIntakeArm(){
        mClimberFrontLead.setPosition(0.0);
    }

    
    public void intakeArmGoToAngle(double angle){
        mClimberFrontLead.setControl(new PositionVoltage(angle * (1/ Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT)));
        //mClimberRearLead.set(0);
    }

    public void intakeArmExtend(){
        intakeArmGoToAngle(Constants.IntakeArmConstants.INTAKE_ARM_INTAKING);
    }


    public void stop(){
        mClimberFrontLead.stopMotor();
    }
}
