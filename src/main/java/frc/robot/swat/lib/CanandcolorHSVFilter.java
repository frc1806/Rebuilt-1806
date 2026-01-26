package frc.robot.swat.lib;

import com.reduxrobotics.sensors.canandcolor.ColorData;

public class CanandcolorHSVFilter {

    private double mMinHue;
    private double mMaxHue;
    private double mMinSat;
    private double mMaxSat;
    private double mMinVal;
    private double mMaxVal;

    /**
     * Check if the colors are in a given HSV range
     * @param minHue minimum Hue value to match
     * @param maxHue maximum Hue value to match
     * @param minSat minimum Saturation value to match
     * @param maxSat maximum Saturation value to match
     * @param minVal minimum Value value to match
     * @param maxVal maximum Value value to match
     */
    public CanandcolorHSVFilter(double minHue, double maxHue, double minSat, double maxSat, double minVal, double maxVal){
        mMinHue = minHue;
        mMaxHue = maxHue;
        mMinSat = minSat;
        mMaxSat = maxSat;
        mMinVal = minVal;
        mMaxVal = maxVal;
    }

    /**
     * 
     * @param colorData
     * @return
     */
    public boolean isInColorRange(ColorData colorData){
        if(mMinHue > mMaxHue){
            //handle wraparound (for exaple filtering for Red.)
            if(colorData.hue() < mMinHue && colorData.hue() > mMaxHue){
                return false;
            }
        }else{
            if(colorData.hue() < mMinHue || colorData.hue() > mMaxHue){
                return false;
        }
        }

        if(colorData.saturation() < mMinSat || colorData.saturation() > mMaxSat){
            return false;
        }
        if(colorData.value() < mMinVal || colorData.value() > mMaxVal){
            return false;
        }
        return true;
    }

}
