package frc.robot.subsystems.swervedrive;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase{

    private SparkMax mClimber1, mClimber2;

    public static ClimberSubsystem GetInstance(){
        return S_INSTANCE;
    }

    private static ClimberSubsystem S_INSTANCE = new ClimberSubsystem();

    private ClimberSubsystem(){
        mClimber1 = new SparkMax(RobotMap.CLIMBER_1, MotorType.kBrushless);
        mClimber2 = new SparkMax(RobotMap.CLIMBER_2, MotorType.kBrushless);
        SparkMaxConfig climberMotorsconfig = new SparkMaxConfig();
        climberMotorsconfig.smartCurrentLimit(60);
        
        mClimber1.configure(climberMotorsconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mClimber2.configure(climberMotorsconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop(){
        mClimber1.set(0);
        mClimber2.set(0);
    }

    public void runClimber(double power1, double power2){
        mClimber1.set(power1);
        mClimber2.set(power2);
    }

    public Command runClimberCommand(DoubleSupplier firstHook, DoubleSupplier secondHook){
        
        return new Command() {
            @Override
            public Set<Subsystem> getRequirements() {
                HashSet<Subsystem> requirements = new HashSet<Subsystem>();
                requirements.add(S_INSTANCE);
                return requirements;
            }
            
            @Override
            public void execute(){
                runClimber(firstHook.getAsDouble(), secondHook.getAsDouble());
            }

        };
    }

}
