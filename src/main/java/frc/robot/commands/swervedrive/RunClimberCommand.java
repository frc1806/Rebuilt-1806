package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RunClimberCommand extends Command{

    DoubleSupplier frontHookDoubleSupplier;

    public RunClimberCommand(DoubleSupplier frontHookOutput){
        addRequirements(RobotContainer.climber);
        frontHookDoubleSupplier = frontHookOutput;
    }

    @Override
    public void execute(){
        RobotContainer.climber.manualControl(frontHookDoubleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.climber.manualControl(0);
    }
}
