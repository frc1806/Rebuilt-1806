// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.swat.lib.CanandcolorHSVFilter;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = Units.lbsToKilograms(110); // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double CANANDCOLOR_GROUND_THRESHOLD = 0.5; //0 - 1, 1 being further away

    public static final CanandcolorHSVFilter BLUE_FILTER = new CanandcolorHSVFilter(.583, .75, 0.5, 1.0, 0.2, 1.0);
    public static final CanandcolorHSVFilter RED_FILTER = new CanandcolorHSVFilter(.9166, 0.083, 0.5, 1.0, 0.2, 1.0);
    
    public static final double BLUE_MIN_HUE = .583;
    public static final double BLUE_MAX_HUE = .75;
    public static final double BLUE_MIN_SAT = 0.5;
    public static final double BLUE_MAX_SAT = 1.0;
    public static final double BLUE_MIN_VAL = 0.2;
    public static final double BLUE_MAX_VAL = 1.0;

    public static final double RED_MIN_HUE = .583;
    public static final double RED_MAX_HUE = .75;
    public static final double RED_MIN_SAT = 0.5;
    public static final double RED_MAX_SAT = 1.0;
    public static final double RED_MIN_VAL = 0.2;
    public static final double RED_MAX_VAL = 1.0;


    public static final double ODOMETRY_STALE_TIME = 2.0;

    public static final Translation2d BLUE_ALLIANCE_GOAL_AIM = new Translation2d(4.833, 4.00);
    public static final Translation2d RED_ALLIANCE_GOAL_AIM = new Translation2d(11.849, 4.00);

    public static final Translation2d RED_ALLIANCE_FEED_AIM = new Translation2d(15.537, 6.907);
    public static final Translation2d BLUE_ALLIANCE_FEED_AIM = new Translation2d(1.0, 1.0);
    public static final double AIM_ANGLE_TOLERANCE = 5;

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double SNAP_ANGLE_RADIUS = 0.6;
  }

  public static class LauncherConstants
  {
    public static final double FLYWHEEL_RPM_TOLERANCE = 300.0;
    public static final double FLYWHEEL_GEAR_RATIO = 1.0; // GEAR UP
    //Fancy 254 style open loop shooting
    public static final int SAMPLES_TO_AVERAGE = 30;
    
    public static final double FLYWHEEL_RAMP_KP = 0.067; // SIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIX SEVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEN
    public static final double FLYWHEEL_RAMP_KI = 0;
    public static final double FLYWHEEL_RAMP_KD = 0;
    public static final double FLYWHEEL_RAMP_KS = 0.06;
    public static final double FLYWHEEL_RAMP_KV = 0.13;

    public static final double FLYWHEEL_PER_MOTOR_SUPPLY_CURRENT_LIMIT = 80;
    public static final double FLYWHEEL_PER_MOTOR_STATOR_CURRENT_LIMIT = 100;

    public static final double FUEL_MOTORS_CURRENT_LIMIT = 80; //TRANSFER & HOPPER
    public static final double FUEL_MOTORS_NOMINAL_VOLTAGE = 10.0; //Motors will output a maximum of 8 volts.

    public static final boolean HOOD_AUTO_ZERO = false;

    public static final double HOOD_MIN_ANGLE = 0.0;
    public static final double HOOD_MAX_ANGLE = 50.0;

    public static final double HOOD_MOTOR_KP = 1.0/10.0;
    public static final double HOOD_MOTOR_KI = 0.0;
    public static final double HOOD_MOTOR_KD = 0.0;

    public static final int SMART_CURRENT_LIMIT = 12;
    public static final double VOLTAGE_COMPENSATION = 8.0;
    public static final boolean HOOD_INVERTED = false;

    public static final double HOOD_GEAR_RATIO = 23.0 * (152.0 / 14.0);

    public static final Angle HOOD_HOME_POSITION =Degree.of(0); //DEGREES
    public static final Angle HOOD_STARTING_POSITION = Degree.of(0);

    public static final double TRANSFER_KP = 0.067; // SIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIX SEVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEN
    public static final double TRANSFER_KI = 0;
    public static final double TRANSFER_KD = 0;
    public static final double TRANSFER_KS = 0.06;
    public static final double TRANSFER_KV = 0.13;

    public static final double TRANSFER_STATOR_CURRENT_LIMIT = 0.0;
    public static final double TRANSFER_SUPPLY_CURRENT_LIMIT = 0.0;

    public static final double TRANSFER_LAUNCHING_RPM = 3500.0;
  }
  public static class CollectorConstants
  {
    public static final double STATOR_CURRENT_LIMIT = 120;
    public static final int SUPPLY_CURRENT_LIMIT = 60;
    public static final int SLIDER_MOTOR_CURRENT_LIMIT = 30;
    public static final double SLIDER_MOTOR_VOLTAGE_COMPENSATION = 8.0;
    public static final boolean SLIDER_MOTOR_INVERTED = true;

    public static final double SLIDER_MOTOR_KP = 10.0/25.0;
    public static final double SLIDER_MOTOR_KI = 0.0;
    public static final double SLIDER_MOTOR_KD = 0.0;

    public static final double OUT_POSITION = 42.0;
    public static final double IN_POSITION =  0.75;

    public static final double INTAKE_ZEROING_CURRENT_THRESHOLD = 5; //amps

    public static final double INTAKE_ZEROING_VELOCITY_THRESHOLD = 5;//RPM

    public static final double SLIDER_SHAKE_POWER_FACTOR = .3;
    public static final double SLIDER_SHAKE_TIME_SCALER = 4;

    public static final double SLIDER_SAFE_EXTENSION_MIN = -10;

  }

  public static class ClimberConstants
  {
    public static final double CLIMBER_STATOR_CURRENT_LIMIT = 120.0;
    public static final double CLIMBER_SUPPLY_CURRENT_LIMIT = 80.0;
    public static final double CLIMBER_L1_GEAR_RATIO = (22.0/14.0)*5.0*5.0*9.0;
    public static final double CLIMBER_L1_DEG_PER_ROT = 360.0 / CLIMBER_L1_GEAR_RATIO;
    public static final double CLIMBER_L1_KP = 4.0;
    public static final double CLIMBER_L1_KI = 0;
    public static final double CLIMBER_L1_KD = 0;

    public static final double CLIMBER_L1_INTAKE_DEFLECT_ANGLE = 165.0;
    public static final double CLIMBER_L1_GRAB_ANGLE = 140.0;
    public static final double CLIMBER_l1_HOOK_ANGLE = 120.0;
    public static final double CLIMBER_L1_CLIMB_ANGLE = 90.0;

  }
}
