package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;

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
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeArmSubsystem extends SubsystemBase{

    private TalonFX mIntakeArmMotor;// mClimberFrontFollower;//, mClimberRearLead; //mClimberRearFollower;
    private SingleJointedArmSim mArmSim;
    private TalonFXSimState mIntakeArmSim;
    private double mTargetAngle;
    private double mIntakeOutVariableOffset;

    public IntakeArmSubsystem(){
        mIntakeArmMotor = new TalonFX(RobotMap.CLIMBER_1);
        mIntakeArmMotor.setNeutralMode(NeutralModeValue.Brake);
        mIntakeOutVariableOffset = 0.0;
        //mClimberFrontFollower = new TalonFX(RobotMap.CLIMBER_2);
        //mClimberRearLead = new TalonFX(RobotMap.CLIMBER_3);
        //mClimberRearFollower = new TalonFX(RobotMap.CLIMBER_4);

        CurrentLimitsConfigs climberCurrentLimitConfig = new CurrentLimitsConfigs();
        climberCurrentLimitConfig.StatorCurrentLimit = Constants.IntakeArmConstants.INTAKE_ARM_STATOR_CURRENT_LIMIT;
        climberCurrentLimitConfig.SupplyCurrentLimit = Constants.IntakeArmConstants.INTAKE_ARM_SUPPLY_CURRENT_LIMIT;

        //Follower frontFollower = new Follower(mClimberFrontLead.getDeviceID(), MotorAlignmentValue.Opposed);

        //Follower reaFollower = new Follower(mClimberRearLead.getDeviceID(), MotorAlignmentValue.Opposed);

        mIntakeArmMotor.getConfigurator().apply(climberCurrentLimitConfig);
        //mClimberFrontFollower.getConfigurator().apply(climberCurrentLimitConfig);
       //mClimberRearLead.getConfigurator().apply(climberCurrentLimitConfig);
        //mClimberRearFollower.getConfigurator().apply(climberCurrentLimitConfig);
        
        MotorOutputConfigs leaderOutputConfigs = new MotorOutputConfigs();
        leaderOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        OpenLoopRampsConfigs mFrontLeadOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
        mFrontLeadOpenLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.25;

        mIntakeArmMotor.getConfigurator().apply(leaderOutputConfigs);
        //mClimberRearLead.getConfigurator().apply(leaderOutputConfigs);

        Slot0Configs l1Slot0Configs = new Slot0Configs();
        l1Slot0Configs.kP = Constants.IntakeArmConstants.INTAKE_ARM_KP;
        l1Slot0Configs.kI = Constants.IntakeArmConstants.INTAKE_ARM_KI;
        l1Slot0Configs.kD = Constants.IntakeArmConstants.INTAKE_ARM_KD;
        mIntakeArmMotor.getConfigurator().apply(l1Slot0Configs);

        //mClimberFrontFollower.setControl(frontFollower);
        //mClimberRearFollower.setControl(reaFollower);
        mArmSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            Constants.IntakeArmConstants.INTAKE_ARM_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(15.0), Units.lbsToKilograms(10.0)),
            Units.inchesToMeters(15.0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(120.0),
            true,
            0,
            Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT,
            0.0);
        mArmSim.setState(Units.degreesToRadians(120.0), 0.0);
            mIntakeArmSim = mIntakeArmMotor.getSimState();
            mIntakeArmSim.setSupplyVoltage(Volts.of(12));
            mTargetAngle = 0;
    }

    @Override
    public void periodic(){
        NetworkTableInstance.getDefault().getTable("Robot/Climber").putValue("Intake Arm Angle", NetworkTableValue.makeDouble(getIntakeArmAngle()));
        NetworkTableInstance.getDefault().getTable("Robot/Climber").putValue("Intake Arm Target Angle", NetworkTableValue.makeDouble(mTargetAngle));
    }

    public void manualControl(double frontOutput){
        if(Math.abs(frontOutput) > 0.25){
            mIntakeArmMotor.set(((Math.abs(frontOutput)-.25)) * (frontOutput <0?-1:1) * 0.5);
        }else{
            mIntakeArmMotor.set(0.0);
        }

        //mClimberRearLead.set(rearOutput * 0.5);
    }

    public double getIntakeArmAngle(){
        return mIntakeArmMotor.getPosition().getValueAsDouble() * Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT;
    }

    public void zeroIntakeArm(){
        mIntakeArmMotor.setPosition(0.0);
    }

    
    public void intakeArmGoToAngle(double angle){
        mIntakeArmMotor.setControl(new PositionVoltage(angle * (1/ Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT)));
        mTargetAngle = angle;
        //mClimberRearLead.set(0);
    }

    public void intakeArmExtend(){
        intakeArmGoToAngle(Constants.IntakeArmConstants.INTAKE_ARM_INTAKING + mIntakeOutVariableOffset);
    }

    public void intakeArmRetract(){
        intakeArmGoToAngle(Constants.IntakeArmConstants.INTAKE_ARM_UP);
    }

    public void intakeArmGoToShootAngle(){
        intakeArmGoToAngle(Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING);
    }

    public void intakeArmOffsetUp(){
        mIntakeOutVariableOffset = mIntakeOutVariableOffset - 2.0;
    }

    public void intakeArmOffsetDown(){
        mIntakeOutVariableOffset = mIntakeOutVariableOffset + 2.0;
    }

     public void stop(){
        mIntakeArmMotor.stopMotor();
    }

    @Override
    public void simulationPeriodic(){
            mArmSim.setInput(-mIntakeArmSim.getMotorVoltage());
            mArmSim.update(0.020);
            mIntakeArmSim.setRawRotorPosition((Units.degreesToRotations(120.0) - (Units.radiansToRotations(mArmSim.getAngleRads())) * Constants.IntakeArmConstants.INTAKE_ARM_GEAR_RATIO));
            mIntakeArmSim.setRotorVelocity(-(Units.radiansPerSecondToRotationsPerMinute(mArmSim.getVelocityRadPerSec()) * Constants.IntakeArmConstants.INTAKE_ARM_GEAR_RATIO) / 60.0);
            NetworkTableInstance.getDefault().getTable("Robot/Climber").putValue("Sim Intake Arm Angle", NetworkTableValue.makeDouble(120.0 - (Units.radiansToDegrees(mArmSim.getAngleRads())*(1/ Constants.IntakeArmConstants.INTAKE_ARM_DEG_PER_ROT))));
            mIntakeArmMotor.setPosition((Units.degreesToRotations(120.0) - (Units.radiansToRotations(mArmSim.getAngleRads())) * Constants.IntakeArmConstants.INTAKE_ARM_GEAR_RATIO));
    }
}
