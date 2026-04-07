package frc.robot.commands.odometrytriggers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForXTripline extends Command{

    private double triggerX;
    private double currentX;
    private double lastX;
    private boolean hasTripped = false;

    public WaitForXTripline(double triplineX){
        this.triggerX = triplineX;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        super.execute();
        currentX = RobotContainer.drivebase.getPose().getX();
        if((lastX <= triggerX && currentX > triggerX) || (currentX <= triggerX && lastX > triggerX) && !hasTripped){
            hasTripped = true;
        }
        lastX = currentX;
    }

    @Override
    public void initialize() {
        super.initialize();
        lastX = RobotContainer.drivebase.getPose().getX();
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
            triggerX = (8.27) + (8.27- triggerX);
        }
    }

    @Override
    public boolean isFinished() {
        return hasTripped;
    }


}
