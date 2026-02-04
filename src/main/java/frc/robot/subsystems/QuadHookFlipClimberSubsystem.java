package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class QuadHookFlipClimberSubsystem extends SubsystemBase {


    SparkMax mFirstHook = new SparkMax(RobotMap.CLIMBER_1, MotorType.kBrushless);
    SparkMax mSecondHook= new SparkMax(RobotMap.CLIMBER_1, MotorType.kBrushless);

    DoubleSupplier firstHookSupplier, secondHookSupplieer;

    private static QuadHookFlipClimberSubsystem S_INSTANCE = new QuadHookFlipClimberSubsystem();


    public static QuadHookFlipClimberSubsystem GetInstance(){
        return S_INSTANCE;
    }
    
    private QuadHookFlipClimberSubsystem(){
        SparkFlexConfig climberMotorsConfig = new SparkFlexConfig();
        climberMotorsConfig.smartCurrentLimit(40);
        climberMotorsConfig.voltageCompensation(8.0);
        climberMotorsConfig.inverted(true);
        mFirstHook.configure(climberMotorsConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mSecondHook.configure(climberMotorsConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void RunClimber(double hook1Power, double hook2Power){
        mFirstHook.set(hook1Power);
        mSecondHook.set(hook2Power);
    }

    public Command getRunClimberCommand(DoubleSupplier firstHook, DoubleSupplier secondHook){
        firstHookSupplier = firstHook;
        secondHookSupplieer = secondHook;
        )
    }
}
