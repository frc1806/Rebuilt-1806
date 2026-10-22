package frc.robot.swat.lib;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import swervelib.SwerveInputStream;

public class AngularVelocityAutoHoldHelper {
    Rotation2d mTargetAngle;

    public void lockAutoHoldAngle(){
        mTargetAngle = RobotContainer.drivebase.getPose().getRotation();
    }

    public DoubleSupplier getXDoubleSupplier(){
        return new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                if(mTargetAngle == null) mTargetAngle = RobotContainer.drivebase.getPose().getRotation();
                return new Translation2d(1.0, mTargetAngle).getY();
            }
            
        };
    }

        public DoubleSupplier getYDoubleSupplier(){
        return new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                if(mTargetAngle == null) mTargetAngle = RobotContainer.drivebase.getPose().getRotation();
                return new Translation2d(1.0, mTargetAngle).getX();
            }
            
        };
    }

}
