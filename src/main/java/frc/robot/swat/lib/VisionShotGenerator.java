package frc.robot.swat.lib;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class VisionShotGenerator {

    private static InterpolatingDoubleTreeMap SHOT_ANGLE_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(0.2, 20.0);
        put(0.3, 22.0);
        put(1.0, 29.0);
        put(1.5, 33.0);
        put(4.0, 45.0);
    }};

    private static InterpolatingDoubleTreeMap SHOT_RPM_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(0.2, 2400.0);
        put(0.3, 2500.0);
        put(1.0, 2650.0);
        put(1.5, 2800.0);
        put(6.0, 3000.0);
    }};

    private static InterpolatingDoubleTreeMap FEED_SPEED_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(0.2, 6.0);
        put(0.3, 6.0);
        put(1.0, 7.0);
        put(1.5, 8.0);
        put(6.0, 9.0);
    }};

    public static Shot GetGoalShotForDistance(double distanceInMeters){
        return new Shot(
            RPM.of(SHOT_RPM_BY_DISTANCE.get(distanceInMeters)), 
            Degree.of(SHOT_ANGLE_BY_DISTANCE.get(distanceInMeters)), 
            Volts.of(FEED_SPEED_BY_DISTANCE.get(distanceInMeters)), 
            true
            );
    }

}
