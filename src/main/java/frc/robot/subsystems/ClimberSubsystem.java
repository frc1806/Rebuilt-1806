package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX mClimberFrontLead, mClimberFrontFollower, mClimberRearLead, mClimberRearFollower;

    public ClimberSubsystem(){
        mClimberFrontLead = new TalonFX(RobotMap.CLIMBER_1);
        mClimberFrontFollower = new TalonFX(RobotMap.CLIMBER_2);
        mClimberRearLead = new TalonFX(RobotMap.CLIMBER_3);
        mClimberRearFollower = new TalonFX(RobotMap.CLIMBER_4);

        CurrentLimitsConfigs climberCurrentLimitConfig = new CurrentLimitsConfigs();
        climberCurrentLimitConfig.StatorCurrentLimit = Constants.ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT;
        climberCurrentLimitConfig.SupplyCurrentLimit = Constants.ClimberConstants.CLIMBER_SUPPLY_CURRENT_LIMIT;

        Follower frontFollower = new Follower(mClimberFrontLead.getDeviceID(), MotorAlignmentValue.Opposed);

        Follower reaFollower = new Follower(mClimberRearLead.getDeviceID(), MotorAlignmentValue.Opposed);

        mClimberFrontLead.getConfigurator().apply(climberCurrentLimitConfig);
        mClimberFrontFollower.getConfigurator().apply(climberCurrentLimitConfig);
        mClimberRearLead.getConfigurator().apply(climberCurrentLimitConfig);
        mClimberRearFollower.getConfigurator().apply(climberCurrentLimitConfig);
        
        MotorOutputConfigs leaderOutputConfigs = new MotorOutputConfigs();
        leaderOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        mClimberFrontLead.getConfigurator().apply(leaderOutputConfigs);
        mClimberRearLead.getConfigurator().apply(leaderOutputConfigs);

        mClimberFrontFollower.setControl(frontFollower);
        mClimberRearFollower.setControl(reaFollower);
    }

    @Override
    public void periodic(){
        
    }

    public void manualControl(double frontOutput, double rearOutput){
        mClimberFrontLead.set(frontOutput);
        mClimberRearLead.set(rearOutput);
    }

}
