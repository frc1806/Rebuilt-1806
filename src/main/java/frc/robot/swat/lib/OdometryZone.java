package frc.robot.swat.lib;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

public class OdometryZone {

    Translation2d bottomLeft, topRight;

    public OdometryZone(Translation2d bottomLeft, Translation2d topRight){
        this.bottomLeft = bottomLeft;
        this.topRight = topRight;
    }

    public boolean isRobotInZone(){
        Translation2d robotTranslation = RobotContainer.drivebase.getPose().getTranslation();
        return (robotTranslation.getX() > bottomLeft.getX()) && (robotTranslation.getX() < topRight.getX()) && (robotTranslation.getY() > bottomLeft.getY()) && (robotTranslation.getY() < topRight.getY());
        
    }



}
