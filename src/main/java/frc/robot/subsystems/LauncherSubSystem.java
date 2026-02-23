package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.swat.lib.Shot;
import frc.robot.swat.lib.VisionShotGenerator;

public class LauncherSubSystem extends SubsystemBase {
    //SINGLETON PATTERN
    private static LauncherSubSystem S_LAUNCHER = new LauncherSubSystem();
    public static LauncherSubSystem GetInstance(){
        return S_LAUNCHER;
    }

    private SwerveSubsystem mDrivebase = RobotContainer.drivebase;
    private MatchTimer mMatchTimer = RobotContainer.matchTimer;

    //Variable Declaration and instantiation if applicable

    //Flywheel Motors

    private TalonFX mFlywheelLeader; 

    private TalonFX mFlywheelFollower; 
    private TalonFX mFlywheelFollowerTwo;

    //Rollers that move the fuel into to the flywheel

    private SparkMax mHoodMotor;
    private SparkMaxConfig mHoodMotorConfig;
    private SoftLimitConfig mHoodMotorSoftLimitConfig;

    //Hopper Motor

    private SparkFlex mHopper;

    private CircularBuffer mFlywheelEstimator = new CircularBuffer<Voltage>(Constants.LauncherConstants.SAMPLES_TO_AVERAGE);

    //Enum for launching states. We're implementing Team 254's 2017 flywheel code but modern.
    enum LauncherStates{
        kIdle, //Doin nothin'
        kClosedLoop, //Spinning up, getting samples
        kOpenLoop, // Open Loop
        kCleaningMode,
        kZeroHood
    }

    private LauncherStates mLauncherState = LauncherStates.kIdle;

    //Stored Target Values
    private AngularVelocity mTargetSpeed = RPM.of(0);
    private VelocityVoltage mFlywheelRequest = new VelocityVoltage(mTargetSpeed);
    private Angle mTargetAngle = Degrees.of(0);
    private Voltage mFeedSpeed = Volt.of(0);
    private Voltage mKf = Volt.of(0);
    private VoltageOut mFlywheelVoltageOut = new VoltageOut(0.0);

    private boolean mEnableLaunch = false;
    private boolean mVisionEnableLaunch = false;

    private static final AngularVelocity kVelocityTolerance = RPM.of(Constants.LauncherConstants.FLYWHEEL_RPM_TOLERANCE);

    //Simulation
    private TalonFXSimState mFlywheelLeaderSim;
    private TalonFXSimState mFlywheelFollowerSim;
    private TalonFXSimState mFlywheelFollowerTwoSim;
    private FlywheelSim mFlywheelSimulation = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(3), 0.0002, 1.0/Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO ), DCMotor.getKrakenX60(3));

    //Constructor is private for singleton pattern
    private LauncherSubSystem(){
        System.out.println("Constructor for Launcher!!!!!!");
        //Make motors exist
        mFlywheelLeader = new TalonFX(RobotMap.LAUNCHER_RIGHT);
        mFlywheelFollower = new TalonFX(RobotMap.LAUNCHER_LEFT);
        mFlywheelFollowerTwo = new TalonFX(RobotMap.LAUNCHER_RIGHT_2);
        mHoodMotor = new SparkMax(RobotMap.HOOD, MotorType.kBrushless);
        mHopper = new SparkFlex(RobotMap.HOPPER, MotorType.kBrushless);

        //SETUP FLYWHEEL MOTORS (Phoenix v6)
        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
        FeedbackConfigs flywheelEncoderConfigs = new FeedbackConfigs();
        flywheelEncoderConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        CurrentLimitsConfigs flywheelCurrentLimitConfigs = new CurrentLimitsConfigs();
        flywheelCurrentLimitConfigs.StatorCurrentLimit = 100;
        flywheelCurrentLimitConfigs.SupplyCurrentLimit = 80;

        Slot0Configs flywheelRampPIDConfig = new Slot0Configs();
        flywheelRampPIDConfig.kP = Constants.LauncherConstants.FLYWHEEL_RAMP_KP;
        flywheelRampPIDConfig.kI = Constants.LauncherConstants.FLYWHEEL_RAMP_KI;
        flywheelRampPIDConfig.kD = Constants.LauncherConstants.FLYWHEEL_RAMP_KD;
        flywheelRampPIDConfig.kS = Constants.LauncherConstants.FLYWHEEL_RAMP_KS;
        flywheelRampPIDConfig.kV = Constants.LauncherConstants.FLYWHEEL_RAMP_KV;

        MotorOutputConfigs leaderOutputConfigs = new MotorOutputConfigs();
        leaderOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        
        //Apply everything for leader
        flywheelLeaderConfig.CurrentLimits = flywheelCurrentLimitConfigs;
        flywheelLeaderConfig.Slot0 = flywheelRampPIDConfig;
        flywheelLeaderConfig.Feedback = flywheelEncoderConfigs;
        flywheelLeaderConfig.MotorOutput = leaderOutputConfigs;
        mFlywheelLeader.getConfigurator().apply(flywheelLeaderConfig);

        //Apply everything for follower
        flywheelFollowerConfig.CurrentLimits = flywheelCurrentLimitConfigs;
        flywheelFollowerConfig.Slot0 = flywheelRampPIDConfig;
        flywheelFollowerConfig.Feedback = flywheelEncoderConfigs;
        mFlywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
        mFlywheelFollowerTwo.getConfigurator().apply(flywheelFollowerConfig);
        //Make follower follow
        mFlywheelFollower.setControl(new Follower(mFlywheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        //END SETUP FLYWHEEEL MOTORS

        //BEGIN SETUP OTHER FUEL MOTORS (RevLib)

        SparkFlexConfig fuelMotorsConfig = new SparkFlexConfig();
        fuelMotorsConfig.smartCurrentLimit(40);
        fuelMotorsConfig.voltageCompensation(8.0);


        fuelMotorsConfig.inverted(false);
        mHopper.configure(fuelMotorsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        //END SETUP OTHER FUEL MOTORS

        //TODO SETUP HOOD MOTOR
        mHoodMotorConfig = new SparkMaxConfig();
        mHoodMotorConfig.smartCurrentLimit(Constants.LauncherConstants.SMART_CURRENT_LIMIT);
        mHoodMotorConfig.voltageCompensation(Constants.LauncherConstants.VOLTAGE_COMPENSATION);
        mHoodMotorConfig.inverted(Constants.LauncherConstants.HOOD_INVERTED);
        mHoodMotorConfig.encoder.positionConversionFactor(Constants.LauncherConstants.HOOD_POSITION_CONVERSION_FACTOR); //Degrees
        mHoodMotorConfig.encoder.velocityConversionFactor(Constants.LauncherConstants.HOOD_POSITION_CONVERSION_FACTOR /60.0); //Degrees per second

        mHoodMotorConfig.closedLoop.pid(Constants.LauncherConstants.HOOD_MOTOR_KP, Constants.LauncherConstants.HOOD_MOTOR_KI, Constants.LauncherConstants.HOOD_MOTOR_KD);
        mHoodMotorConfig.closedLoop.maxMotion.cruiseVelocity(180.0);
        mHoodMotorConfig.closedLoop.maxMotion.maxAcceleration(45.0);
        mHoodMotorSoftLimitConfig = new SoftLimitConfig();
        mHoodMotorSoftLimitConfig.forwardSoftLimit(Constants.LauncherConstants.HOOD_MAX_ANGLE);
        mHoodMotorSoftLimitConfig.forwardSoftLimitEnabled(true);
        mHoodMotorSoftLimitConfig.reverseSoftLimit(Constants.LauncherConstants.HOOD_MIN_ANGLE);
        mHoodMotorSoftLimitConfig.reverseSoftLimitEnabled(true);
        mHoodMotorConfig.apply(mHoodMotorSoftLimitConfig);

        mHoodMotor.configure(mHoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        //SIM
        
        mFlywheelLeaderSim = new TalonFXSimState(mFlywheelLeader);
        mFlywheelLeaderSim.setSupplyVoltage(12.0);
        mFlywheelLeaderSim.setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);
        mFlywheelFollowerSim = new TalonFXSimState(mFlywheelFollower);
        mFlywheelFollowerSim.setSupplyVoltage(12.0);
        mFlywheelFollowerSim.setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);
        mFlywheelFollowerTwoSim = new TalonFXSimState(mFlywheelFollowerTwo);
        mFlywheelFollowerTwoSim.setSupplyVoltage(12.0);
        mFlywheelFollowerTwoSim.setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);


        if(Constants.LauncherConstants.HOOD_AUTO_ZERO){
            zeroHood();
        }
    }

    /**
     * Shoot fuel with the given parameters
     * @param speed speed of the flywheel
     * @param angle angle of the shooter hood
     * @param feedSpeed speed of the transfer wheels, voltage.
     */

    public void shoot(AngularVelocity speed, Angle angle, Voltage feedSpeed){
        if(!speed.isNear(mTargetSpeed, Constants.LauncherConstants.FLYWHEEL_RPM_TOLERANCE))
        {
            mFlywheelEstimator.clear();
            mLauncherState = LauncherStates.kClosedLoop;
        }
        mTargetSpeed = speed;
        mFlywheelRequest = new VelocityVoltage(mTargetSpeed.div(Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO)); //Saving RAM by only instantiating this on a change.
        mTargetAngle = angle;
        mFeedSpeed = feedSpeed;
        if(mLauncherState != LauncherStates.kClosedLoop && mLauncherState != LauncherStates.kOpenLoop)
        {
            mLauncherState = LauncherStates.kClosedLoop;
        } 
    }

        /**
         * 
         * @param shot
         */
        public void shoot(Shot shot){
            shoot(shot.getFlywheelSpeed(), shot.getHoodAngle(), shot.getFeedSpeed());
        }


    /**
     * Gets the flywheel velocity and scales for the Gear (UP) Ratio
     * @return {@link AngularVelocity} of the flywheel, not the motor speed.
     */
    public AngularVelocity getFlywheelVelocity(){
        return RPM.of(RPM.convertFrom(mFlywheelLeader.getVelocity().getValue().magnitude(), RotationsPerSecond)).times(Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO);
    }

    /**
     * Shut down everything
     */
    public void stop(){
        mFlywheelLeader.stopMotor();
        //#jacobtodo
        mHopper.stopMotor();
        mLauncherState = LauncherStates.kIdle;
        mTargetSpeed = RPM.of(0);
        mFlywheelRequest = new VelocityVoltage(RPM.of(0)); //Not needed, but to be safe.
        mTargetAngle = Angle.ofBaseUnits(0, Degrees);
    }

    public boolean isAtSpeed(){
            final AngularVelocity currentVelocity = getFlywheelVelocity();
            return currentVelocity.isNear(mTargetSpeed, kVelocityTolerance) && mTargetSpeed.gt(RPM.of(0));
    }

    public boolean isAtAngle(){
        return true; //TODO: Implement hood
    }

    /**
     * Estimate voltage output to RPM ratio. RPM because we will manually set it as a Voltage out later.
     * @return
     */
    private Voltage estimatekF(){
        return Voltage.ofBaseUnits((mFlywheelLeader.getMotorVoltage().getValueAsDouble() / getFlywheelVelocity().in(RPM)), Volt);
    }

    @SuppressWarnings("unchecked")
    private void savekF(){
        Voltage total = Voltage.ofBaseUnits(0.0, Volt);
        for(int i=0; i < mFlywheelEstimator.size(); i++){
            total = total.plus((Voltage) mFlywheelEstimator.get(i));
        }
        mKf = total.div(mFlywheelEstimator.size());
        mFlywheelVoltageOut = new VoltageOut(mKf.magnitude() * mTargetSpeed.in(RPM));
    }

    /**
     * Start allowing fuel to leave the robot if the flywheel is at speed.
     */
    public void enableLaunching(){
        mEnableLaunch = true;
    }

    /**
     * Stop allowing fuel to leave the robot, even if the flywheel is at speed.
     */
    public void disableLaunching(){
        mEnableLaunch = false;
    }


    @SuppressWarnings("unchecked")
    @Override
    public void periodic() {
        super.periodic();

        //STATE MACHINE
        switch(mLauncherState){
            case kClosedLoop:
                mHoodMotor.getClosedLoopController().setSetpoint(mTargetAngle.in(Degrees), ControlType.kMAXMotionPositionControl);
                mFlywheelLeader.setControl(mFlywheelRequest); 
                mHopper.stopMotor();
                if(isAtSpeed())
                {
                    mFlywheelEstimator.addLast(estimatekF());
                    if(mFlywheelEstimator.size() >= Constants.LauncherConstants.SAMPLES_TO_AVERAGE && (mEnableLaunch || getSmartEnableLaunch()))
                    {
                        savekF();
                        mLauncherState = LauncherStates.kOpenLoop;
                    }
                }

                break;
            case kOpenLoop:
                mHoodMotor.getClosedLoopController().setSetpoint(mTargetAngle.in(Degrees), ControlType.kMAXMotionPositionControl);
                mFlywheelLeader.setControl(mFlywheelVoltageOut);
                if(!isAtSpeed()){
                    estimatekF();
                    savekF();
                }
                if(!mEnableLaunch && !getSmartEnableLaunch()){
                    mLauncherState = LauncherStates.kClosedLoop;
                }
                mHopper.setVoltage(mFeedSpeed);
                break;
            case kCleaningMode:
                 mHoodMotor.getClosedLoopController().setSetpoint(Constants.LauncherConstants.HOOD_HOME_POSITION.in(Degrees), ControlType.kMAXMotionPositionControl);
                //DO Nothing for cleaning mode, managed externally
            break;
            case kZeroHood:
                mHoodMotor.set(-.1);
                mFlywheelLeader.set(0);
                mHopper.set(0);
                if(Math.abs(mHoodMotor.getOutputCurrent()) > 2 && Math.abs(mHoodMotor.getEncoder().getVelocity()) < 2.0)
                {
                    mHoodMotor.getEncoder().setPosition(0);
                    mHoodMotor.set(0);
                    mLauncherState = LauncherStates.kIdle;
                    mHoodMotorSoftLimitConfig.forwardSoftLimitEnabled(true);
                    mHoodMotorSoftLimitConfig.reverseSoftLimitEnabled(true);
                    mHoodMotorConfig.apply(mHoodMotorSoftLimitConfig);
                    mHoodMotor.configure(mHoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                }
            break;
            case kIdle:  //Intentionally no-break after kIdle, we want kIdle to be effectively the default
            default:
                mHoodMotor.getClosedLoopController().setSetpoint(Constants.LauncherConstants.HOOD_HOME_POSITION.in(Degrees), ControlType.kMAXMotionPositionControl);
                mFlywheelLeader.stopMotor();
                mHopper.stopMotor();
                break;
            
        }

        SmartDashboard.putNumber("Launcher/RPM", getFlywheelVelocity().magnitude());
        SmartDashboard.putNumber("Launcher/Angle", mHoodMotor.getEncoder().getPosition());
        SmartDashboard.putString("Launcher/State", mLauncherState.name());
        SmartDashboard.putNumber("Launcher/Target RPM", mTargetSpeed.magnitude());
        SmartDashboard.putNumber("Launcher/TargetAngle", mTargetAngle.in(Degrees));
        SmartDashboard.putNumber("Launcher/kF", mKf.magnitude());
        SmartDashboard.putNumber("Launcher/kF Samples", mFlywheelEstimator.size());
        SmartDashboard.putNumber("Launcher/Sim/SimSpeed", mFlywheelSimulation.getAngularVelocityRPM());
        SmartDashboard.putNumber("Launcher/Sim/SimAmps", mFlywheelSimulation.getCurrentDrawAmps());
        SmartDashboard.putNumber("Launcher/Sim/SimInputVolts", mFlywheelSimulation.getInputVoltage());
        SmartDashboard.putNumber("Launcher/Sim/SimKrakenMotorVolts", mFlywheelLeaderSim.getMotorVoltage());

    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        mFlywheelSimulation.setInputVoltage(mFlywheelLeaderSim.getMotorVoltage());
        mFlywheelSimulation.update(0.02);
        mFlywheelLeaderSim.setRotorVelocity((mFlywheelSimulation.getAngularVelocityRPM() / Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO) / 60.0);
        mFlywheelFollowerSim.setRotorVelocity((mFlywheelSimulation.getAngularVelocityRPM() / Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO) / 60.0);
        mFlywheelFollowerTwoSim.setRotorVelocity((mFlywheelSimulation.getAngularVelocityRPM() / Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO) / 60.0);
    }

    /**
     * Shoot fuel with the given parameters
     * @param speed speed of the flywheel
     * @param angle angle of the shooter hood
     * @param feedSpeed speed of the transfer wheels, voltage.
     * @return a Command that will start ramping the flywheel.
     */
    public Command prepareShotCommand(AngularVelocity speed, Angle angle, Voltage feedSpeed){
        return (Commands.run(
            new Runnable(){
                @Override
                public void run() {
                    shoot(speed, angle, feedSpeed);
                }
            }, this)
        );
    }

/**
 * 
 * @param shot
 * @return
 */
    public Command prepareShotCommand(Shot shot){
        return prepareShotCommand(shot.getFlywheelSpeed(), shot.getHoodAngle(), shot.getFeedSpeed());
    }

    public void clean(){
        mLauncherState = LauncherStates.kCleaningMode;
        mFlywheelLeader.setVoltage(3.0);
        mHopper.setVoltage(3.0);
    }

    public void adjustShooterOffsetHigher(){
        RobotPreferences.SetShooterAngleOffset(RobotPreferences.GetShooterAngleOffset() + .1);
    }
    
    public void adjustShooterOffsetLower(){
        RobotPreferences.SetShooterAngleOffset(RobotPreferences.GetShooterAngleOffset() - .1);
    }

    public void zeroHood(){
        mLauncherState = LauncherStates.kZeroHood;
        mHoodMotorSoftLimitConfig.forwardSoftLimitEnabled(false);
        mHoodMotorSoftLimitConfig.reverseSoftLimitEnabled(false);
        mHoodMotorConfig.apply(mHoodMotorSoftLimitConfig);
        mHoodMotor.configure(mHoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    public boolean getVisionEnableLaunch(){
        return Math.abs(Units.radiansToDegrees(mDrivebase.getAngleToTargetRadians())) < Math.max(Constants.DrivebaseConstants.AIM_ANGLE_TOLERANCE - mDrivebase.getDistanceToTarget(), 0.2);
    }

    public boolean getSmartEnableLaunch(){
        return getVisionEnableLaunch() && mMatchTimer.getHubState().mIsActive;
    }

    public Command launcherAimAtGoal(){
        return new Command(){

 @Override
            public Set<Subsystem> getRequirements() {
                HashSet<Subsystem> reqs = new HashSet<>();
                reqs.add(RobotContainer.launcher);
                return reqs;
            }

            @Override
            public void execute(){
                shoot(VisionShotGenerator.GetGoalShotForDistance(mDrivebase.getDistanceToTarget()));
            };
        };
    }
}
