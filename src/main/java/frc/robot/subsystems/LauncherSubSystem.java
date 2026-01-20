package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LauncherSubSystem extends SubsystemBase {

    private TalonFX mFlywheelLeader = new TalonFX(RobotMap.LAUNCHER_LEFT);

    private TalonFX mFlywheelFollower = new TalonFX(RobotMap.LAUNCHER_RIGHT);

    private SparkMax mTransfer = new SparkMax(RobotMap.TRANSFER, MotorType.kBrushless);

    private SparkMax mHopper = new SparkMax(RobotMap.HOPPER, MotorType.kBrushless);

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();
    }

}
