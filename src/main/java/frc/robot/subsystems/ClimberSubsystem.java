package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX mClimberFrontLead, mClimberFrontFollower;//, mClimberRearLead; //mClimberRearFollower;

    public ClimberSubsystem(){
        mClimberFrontLead = new TalonFX(RobotMap.CLIMBER_1);
        mClimberFrontFollower = new TalonFX(RobotMap.CLIMBER_2);
        //mClimberRearLead = new TalonFX(RobotMap.CLIMBER_3);
        //mClimberRearFollower = new TalonFX(RobotMap.CLIMBER_4);

        CurrentLimitsConfigs climberCurrentLimitConfig = new CurrentLimitsConfigs();
        climberCurrentLimitConfig.StatorCurrentLimit = Constants.ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT;
        climberCurrentLimitConfig.SupplyCurrentLimit = Constants.ClimberConstants.CLIMBER_SUPPLY_CURRENT_LIMIT;

        Follower frontFollower = new Follower(mClimberFrontLead.getDeviceID(), MotorAlignmentValue.Opposed);

        //Follower reaFollower = new Follower(mClimberRearLead.getDeviceID(), MotorAlignmentValue.Opposed);

        mClimberFrontLead.getConfigurator().apply(climberCurrentLimitConfig);
        mClimberFrontFollower.getConfigurator().apply(climberCurrentLimitConfig);
       //mClimberRearLead.getConfigurator().apply(climberCurrentLimitConfig);
        //mClimberRearFollower.getConfigurator().apply(climberCurrentLimitConfig);
        
        MotorOutputConfigs leaderOutputConfigs = new MotorOutputConfigs();
        leaderOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        OpenLoopRampsConfigs mFrontLeadOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
        mFrontLeadOpenLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.25;

        mClimberFrontLead.getConfigurator().apply(leaderOutputConfigs);
        //mClimberRearLead.getConfigurator().apply(leaderOutputConfigs);

        Slot0Configs l1Slot0Configs = new Slot0Configs();
        l1Slot0Configs.kP = Constants.ClimberConstants.CLIMBER_L1_KP;
        l1Slot0Configs.kI = Constants.ClimberConstants.CLIMBER_L1_KI;
        l1Slot0Configs.kD = Constants.ClimberConstants.CLIMBER_L1_KD;
        mClimberFrontLead.getConfigurator().apply(l1Slot0Configs);

        mClimberFrontFollower.setControl(frontFollower);
        //mClimberRearFollower.setControl(reaFollower);
    }

    @Override
    public void periodic(){
        NetworkTableInstance.getDefault().getTable("Robot/Climber").putValue("L1 Angle", NetworkTableValue.makeDouble(getL1Angle()));
        
    }

    public void manualControl(double frontOutput, double rearOutput){
        mClimberFrontLead.set(frontOutput * 0.5);
        //mClimberRearLead.set(rearOutput * 0.5);
    }

    public double getL1Angle(){
        return mClimberFrontLead.getPosition().getValueAsDouble() * Constants.ClimberConstants.CLIMBER_L1_DEG_PER_ROT;
    }

    public void l1GoToAngle(double angle){
        mClimberFrontLead.setControl(new PositionVoltage(angle * (1/ Constants.ClimberConstants.CLIMBER_L1_DEG_PER_ROT)));
        //mClimberRearLead.set(0);
    }

    public void zeroL1(){
        mClimberFrontLead.setPosition(0.0);
    }


}
