package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Collector extends SubsystemBase {

private TalonFX mCollectorMotor = new TalonFX(RobotMap.INTAKE);

private static Collector S_COLLECTOR = new Collector();
public static Collector GetInstance(){
    return S_COLLECTOR;
}

private Collector(){
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.CollectorConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.CollectorConstants.SUPPLY_CURRENT_LIMMIT)).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    mCollectorMotor.getConfigurator().apply(rollerConfig);

}



public void stop(){
    mCollectorMotor.stopMotor();
} 

public void intake(){
    System.out.println("Running intake");
    mCollectorMotor.setVoltage(12.0);
}

public void outtake(){
    mCollectorMotor.setVoltage(-12.0);
}

}
