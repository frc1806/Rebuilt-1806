package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ClimberL1GoToAngleCommand extends Command {
    
    private double mAngle;
    public ClimberL1GoToAngleCommand(double angle){
        addRequirements(RobotContainer.climber);
        mAngle = angle;
    }

    @Override
    public void execute(){
        RobotContainer.climber.l1GoToAngle(mAngle);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(RobotContainer.climber.getL1Angle() - mAngle) < 1.0;

    }

    public void end(boolean interrupted){
        RobotContainer.climber.manualControl(0.0);
    }
}
