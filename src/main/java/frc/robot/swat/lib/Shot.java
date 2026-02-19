package frc.robot.swat.lib;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Shot {
    private AngularVelocity mFlywheelSpeed; 
    private Angle mHoodAngle; 
    private Voltage mFeedSpeed;
    private boolean mPreciseShot;

    
    public AngularVelocity getFlywheelSpeed() {
        return mFlywheelSpeed;
    }


    public Angle getHoodAngle() {
        return mHoodAngle;
    }


    public Voltage getFeedSpeed() {
        return mFeedSpeed;
    }


    public boolean isPreciseShot() {
        return mPreciseShot;
    }

    /**
     * Create a shot object
     * @param flywheelSpeed the speed of the launcher wheel
     * @param hoodAngle the angle of the hood
     * @param feedSpeed the hopper and the transfer wheels
     * @param preciseShot whether or not we are shooting into the goal
     */
    public Shot(AngularVelocity flywheelSpeed, Angle hoodAngle, Voltage feedSpeed, boolean preciseShot) {
        this.mFlywheelSpeed = flywheelSpeed;
        this.mHoodAngle = hoodAngle;
        this.mFeedSpeed = feedSpeed;
        this.mPreciseShot = preciseShot;
    }


}
