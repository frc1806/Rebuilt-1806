// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.LauncherSubSystem;
import frc.robot.subsystems.MatchTimer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.swat.lib.Shot;
import frc.robot.swat.lib.SnapAnglesHelper;
import frc.robot.swat.lib.SnapAnglesHelper.FieldSnapAngles;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controlleror  CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);

  public static final Shot CLOSE_SHOT = new Shot(RPM.of(2000), Degrees.of(0.0), Volts.of(8.0), true);
  public static final Shot PROTECTED_SHOT = new Shot(RPM.of(2800), Degrees.of(43.0), Volts.of(6.0), true);

  //Test Modes
  public enum TestModes{
    kFlywheelTest,
    kIntakeTest,
    kClimberTest,
    kHopperTest,
    kCleaningMode,
    kDriveTest,
    kHoodTestMode
  }
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  public static final MatchTimer matchTimer = new MatchTimer(); //has to be ahead of launcher being init.

  public static final LauncherSubSystem launcher = LauncherSubSystem.GetInstance();
  public static final Collector collector = Collector.GetInstance();


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SnapAnglesHelper snapAnglesHelper = new SnapAnglesHelper(FieldSnapAngles.k2026RebuiltAngles).withAllianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(snapAnglesHelper.getXDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1), snapAnglesHelper.getYDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1)).headingWhile(true);

    SwerveInputStream driveDirectAngleSlow = driveAngularVelocity.copy().scaleTranslation(.5).withControllerHeadingAxis(snapAnglesHelper.getXDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1), snapAnglesHelper.getYDoubleSupplier(() ->driverXbox.getRightX() * -1,
  () -> driverXbox.getRightY() * -1)).headingWhile(true);


  SwerveInputStream driveGoalAim = driveAngularVelocity.copy().withControllerHeadingAxis(
    ()-> (drivebase.isRedAlliance()? Constants.DrivebaseConstants.RED_ALLIANCE_GOAL_AIM:Constants.DrivebaseConstants.BLUE_ALLIANCE_GOAL_AIM).minus(drivebase.getPose().getTranslation()).getY(), 
    ()-> (drivebase.isRedAlliance()? Constants.DrivebaseConstants.RED_ALLIANCE_GOAL_AIM:Constants.DrivebaseConstants.BLUE_ALLIANCE_GOAL_AIM).minus(drivebase.getPose().getTranslation()).getX()).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  Command driveSpeen = drivebase.driveSpeen(driveAngularVelocity, new Translation2d(Units.inchesToMeters(13.5 + 12), 0));
  Command driveBeyblade = drivebase.driveSpeen(driveAngularVelocity, new Translation2d(Units.inchesToMeters(6), 0));





  // 2. Create a reusable path builder
  FollowPath.Builder pathBuilder = new FollowPath.Builder(
      drivebase,
      drivebase::getPose,
      drivebase::getRobotVelocity,
      drivebase::drive,
      new PIDController(1.5, 0.00, 0.1),  // translation
      new PIDController(3.0, 0.0, 0.0),  // rotation
      new PIDController(1.0, 0.0, 0.0)   // cross-track
  ).withDefaultShouldFlip()
  .withPoseReset(drivebase::resetOdometry);

  private SendableChooser<TestModes> mTestModeChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    //Populate Test Modes
    for(TestModes testMode : TestModes.values())
    {
      mTestModeChooser.addOption(testMode.name(), testMode);
    }
    SmartDashboard.putData("test/testmodes",mTestModeChooser);
    mTestModeChooser.onChange(this::setTestModeBindings);

    new Trigger(() -> DriverStation.isTeleopEnabled() && (!matchTimer.isRunning() || matchTimer.isTeleopTimerExpired())).onTrue(Commands.runOnce(matchTimer::startTimer));
    new Trigger(() -> DriverStation.isAutonomous() && matchTimer.isRunning()).onTrue(Commands.runOnce(matchTimer::stopTimer));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  public void configureBindings()
  {
    
      driverXbox.povUp().onTrue(Commands.runOnce(launcher::adjustShooterOffsetLower));
      driverXbox.povDown().onTrue(Commands.runOnce(launcher::adjustShooterOffsetHigher));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    Command driveGoalAimCommand = drivebase.driveFieldOriented(driveGoalAim);

    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    collector.setDefaultCommand(Commands.run(collector::stop, collector));
    launcher.setDefaultCommand(Commands.run(launcher::stop, launcher));

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      /*driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
                                                      */

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if(DriverStation.isTest()){
      setTestModeBindings(mTestModeChooser.getSelected());
    }
else
    {
      
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.y().onTrue(launcher.prepareShotCommand(CLOSE_SHOT));
      driverXbox.y().or(driverXbox.b()).onFalse(Commands.runOnce(launcher::stop, launcher));
      driverXbox.b().onTrue(launcher.prepareShotCommand(PROTECTED_SHOT));
      driverXbox.rightTrigger().onTrue(Commands.runOnce(launcher::enableLaunching));
      driverXbox.rightTrigger().onFalse(Commands.runOnce(launcher::disableLaunching));
      driverXbox.leftTrigger().whileTrue(Commands.run(collector::intake, collector));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.leftBumper().whileTrue(driveSpeen);
      driverXbox.rightBumper().whileTrue(driveBeyblade);

      driverXbox.povUp().onTrue(Commands.runOnce(launcher::adjustShooterOffsetLower));
      driverXbox.povDown().onTrue(Commands.runOnce(launcher::adjustShooterOffsetHigher));
      driverXbox.a().whileTrue(driveGoalAimCommand.alongWith(launcher.launcherAimAtGoal()));
      

    }

  }

  private void setTestModeBindings(TestModes mode){
    if (DriverStation.isTest())
    {
      //Precaution
      //launcher.stop();
      drivebase.stop();
      collector.stop();
      drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
      //launcher.setDefaultCommand(Commands.run(launcher::stop, launcher));
      collector.setDefaultCommand(Commands.run(collector::stop, collector));


      if(mode == null){
        System.out.println("Tried to bind null test mode.");
        return;
        
      }
      System.out.println(mTestModeChooser.getSelected().name());
      switch(mode){
        case kCleaningMode:

            drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
            collector.setDefaultCommand(Commands.run(collector::clean, collector));
            launcher.setDefaultCommand(Commands.run(launcher::clean, launcher));
            driverXbox.a().or(operatorXbox.a()).whileTrue(new ParallelCommandGroup(Commands.run(collector::stop, collector), Commands.run(launcher::stop, launcher)));
          break;
        case kClimberTest:/*
            drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
            climber.setDefaultCommand(climber.runClimberCommand(
              new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return Math.abs(operatorXbox.getLeftY()) > OperatorConstants.DEADBAND? operatorXbox.getLeftY(): 0;
              };
            }, 
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return Math.abs(operatorXbox.getRightY()) > OperatorConstants.DEADBAND? operatorXbox.getRightY(): 0;
              };
            }));*/
          break;
        case kFlywheelTest:
          drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
          break;
        case kHopperTest:
          drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
          launcher.enableLaunching();
          driverXbox.a().onTrue(launcher.prepareShotCommand(RPM.of(500), Degrees.of(0), Voltage.ofBaseUnits(8, Volts)));
          driverXbox.a().onFalse(Commands.run(launcher::stop, launcher));
          break;
        case kIntakeTest:
          drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
          driverXbox.leftTrigger().whileTrue(Commands.run(collector::intake, collector));
          driverXbox.a().onTrue(Commands.runOnce(collector::zeroExtending));
          driverXbox.b().onTrue(Commands.runOnce(collector::retract));
          break;
        case kDriveTest:
          drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
          break;
        case kHoodTestMode:
            //drivebase.setDefaultCommand(Commands.run(drivebase::stop));
            driverXbox.a().onTrue(Commands.runOnce(launcher::testHood15));
            driverXbox.b().onTrue(Commands.runOnce(launcher::testHood30));
            driverXbox.y().onTrue(Commands.runOnce(launcher::testHood45));
            driverXbox.x().onTrue(Commands.runOnce(launcher::zeroHood));
            break;
        default:
          drivebase.setDefaultCommand(Commands.run(drivebase::stop, drivebase));
          break;
        
      }

    } 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");

      // 3. Load and follow a path
  Path myPath = new Path("SweepRight");
  Command followCommand = pathBuilder.build(new Path("SweepRight")).andThen(pathBuilder.build(new Path("SweepRight2")).andThen(Commands.runOnce(drivebase::stop, drivebase)));
  return followCommand;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void disabledPeriodic(){
    matchTimer.writeToNetworkTables();;
  }
}
