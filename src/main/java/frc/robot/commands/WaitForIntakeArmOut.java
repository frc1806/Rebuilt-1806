package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForIntakeArmOut extends Command{

    @Override 
    public boolean isFinished() {
        return RobotContainer.intakeArm.getIntakeArmAngle() > 45.0;
}
}