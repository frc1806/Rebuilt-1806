package frc.robot.swat.lib;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class VisionShotGenerator {

    private static InterpolatingDoubleTreeMap SHOT_ANGLE_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(1.5, 6.0);
        put(2.0, 12.0);
        put(2.5, 17.0);
        put(3.0, 22.0);
        put(4.0, 25.0);
        put (5.0, 29.0);

    }};

    private static InterpolatingDoubleTreeMap SHOT_RPM_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(1.5, 3550.0);
        put(2.0, 3900.0);
        put(2.5, 4300.0);
        put(3.0, 4500.0);
        put(4.0, 5000.0);
        put(5.0, 6000.0);
    }};

    private static InterpolatingDoubleTreeMap FEED_SPEED_BY_DISTANCE = new InterpolatingDoubleTreeMap()
    {{
        put(1.5, 6.0);
        put(2.0, 7.0);
        put(2.5, 8.0);
        put(3.0, 10.0);
        put(4.0, 11.0);
        put(5.0, 12.0);

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
