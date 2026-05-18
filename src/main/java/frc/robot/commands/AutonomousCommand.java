package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/**
 * Wrapper for autonomous commands with extra safeties for cancelling itself and stopping things when it's done.
 */

public class AutonomousCommand extends Command{

    private Command mAutonomousCommand;

    public AutonomousCommand(Command autoCommand){
        mAutonomousCommand = new WaitCommand(0.05).andThen(autoCommand);
        addRequirements(autoCommand.getRequirements());
    }



    @Override
    public void cancel() {
        super.cancel();
        mAutonomousCommand.cancel();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        mAutonomousCommand.end(interrupted);
        //Make sure it's ready for tele.
        RobotContainer.launcher.stop();
        RobotContainer.drivebase.stop();
        RobotContainer.collector.stop();
        RobotContainer.climber.stop();
        RobotContainer.launcher.disableLaunching();
        RobotContainer.snapAnglesHelper.clearSnapAngleMemory();
    }

    @Override
    public void execute() {
        super.execute();
        mAutonomousCommand.execute();
    }

    @Override
    public void initialize() {
        super.initialize();
        mAutonomousCommand.initialize();
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isTeleopEnabled())
        {
            return true;
        }
        return mAutonomousCommand.isFinished();
    }

}
