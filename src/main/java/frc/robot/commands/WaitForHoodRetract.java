package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForHoodRetract  extends Command{

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return RobotContainer.launcher.getHoodAngle().in(Degrees) < 2.0;
    }

}
