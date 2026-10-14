package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RaiseIntakeArmWhileShooting extends Command {
    
    double mLaunchingTime;
    public RaiseIntakeArmWhileShooting(){
        addRequirements(RobotContainer.intakeArm);
        mLaunchingTime = 0.0;
    }

    @Override
    public void execute(){
        mLaunchingTime = RobotContainer.launcher.getLaunchingTime();
        if(mLaunchingTime > 5.0)
        {
            RobotContainer.intakeArm.intakeArmRetract();
        }
        else if (mLaunchingTime > Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING_START_RAISE_TIME && mLaunchingTime <= Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING_END_RAISE_TIME)
        {
            RobotContainer.intakeArm.intakeArmGoToAngle(Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING 
                                                - (Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING * 
                                                ((mLaunchingTime - Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING_START_RAISE_TIME)/ (Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING_END_RAISE_TIME - Constants.IntakeArmConstants.INTAKE_ARM_SHOOTING_START_RAISE_TIME ))
                                                ));
        }
        else
        {
            RobotContainer.intakeArm.intakeArmExtend();
        }
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.intakeArm.intakeArmExtend();
    }


}
