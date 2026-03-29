package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.HashSet;
import java.util.Set;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
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

    private TalonFX mHoodMotor;
    private TalonFXConfiguration mHoodMotorConfig;
    private SoftwareLimitSwitchConfigs mHoodMotorSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    private double mStartLaunchingTime =0;

    //Hopper Motor

    private TalonFX mHopper;

    private TalonFX mTransfer;








    private CircularBuffer mFlywheelEstimator = new CircularBuffer<Voltage>(Constants.LauncherConstants.SAMPLES_TO_AVERAGE);

    //Enum for launching states. We're implementing Team 254's 2017 flywheel code but modern.
    enum LauncherStates{
        kIdle, //Doin nothin'
        kClosedLoop, //Spinning up, getting samples
        kOpenLoop, // Open Loop
        kCleaningMode,
        kZeroHood,
        kTestHood
    }

    private LauncherStates mLauncherState = LauncherStates.kIdle;

    //Stored Target Values
    private AngularVelocity mTargetSpeed = RPM.of(0);
    private VelocityVoltage mFlywheelRequest = new VelocityVoltage(mTargetSpeed);
    private VelocityVoltage mTransferMoveRequest = new VelocityVoltage(RPM.of(Constants.LauncherConstants.TRANSFER_LAUNCHING_RPM));
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
        //System.out.println("Constructor for Launcher!!!!!!");
        //Make motors exist
        mFlywheelLeader = new TalonFX(RobotMap.LAUNCHER_RIGHT);
        mFlywheelFollower = new TalonFX(RobotMap.LAUNCHER_LEFT);
        mFlywheelFollowerTwo = new TalonFX(RobotMap.LAUNCHER_RIGHT_2);
        mHoodMotor = new TalonFX(RobotMap.HOOD);
        mHopper = new TalonFX(RobotMap.HOPPER);
        mTransfer = new TalonFX(RobotMap.TRANSFER);

        //SETUP FLYWHEEL MOTORS (Phoenix v6)
        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
        FeedbackConfigs flywheelEncoderConfigs = new FeedbackConfigs();
        flywheelEncoderConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flywheelEncoderConfigs.SensorToMechanismRatio = 1 / Constants.LauncherConstants.FLYWHEEL_GEAR_RATIO;

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
/* 
        SparkFlexConfig fuelMotorsConfig = new SparkFlexConfig();
        fuelMotorsConfig.smartCurrentLimit(40);
        fuelMotorsConfig.voltageCompensation(8.0);


        fuelMotorsConfig.inverted(true);
        mHopper.configure(fuelMotorsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        */
        CurrentLimitsConfigs hopperCurrentLimitsConfigs = new CurrentLimitsConfigs();
        hopperCurrentLimitsConfigs.StatorCurrentLimit = 100;
        hopperCurrentLimitsConfigs.SupplyCurrentLimit = 60;

        MotorOutputConfigs hoppeMotorOutputConfigs = new MotorOutputConfigs();
        hoppeMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        mHopper.getConfigurator().apply(hopperCurrentLimitsConfigs);
        mHopper.getConfigurator().apply(hoppeMotorOutputConfigs);
        //END SETUP OTHER FUEL MOTORS

        mHoodMotorConfig = new TalonFXConfiguration();
        mHoodMotor.setNeutralMode(NeutralModeValue.Brake);
        mHoodMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        FeedbackConfigs HoodMotorEncoderConfigs = new FeedbackConfigs();
        HoodMotorEncoderConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodMotorEncoderConfigs.SensorToMechanismRatio = Constants.LauncherConstants.HOOD_GEAR_RATIO;
        mHoodMotorConfig.Feedback = HoodMotorEncoderConfigs;
        mHoodMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
        mHoodMotorSoftLimitConfig.ForwardSoftLimitThreshold = Rotation.convertFrom(Constants.LauncherConstants.HOOD_MAX_ANGLE, Degree);
        mHoodMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
        mHoodMotorSoftLimitConfig.ReverseSoftLimitThreshold = Rotation.convertFrom(Constants.LauncherConstants.HOOD_MIN_ANGLE, Degree);
        mHoodMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
        mHoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 16;

        Slot0Configs HoodMotorPIDConfig = new Slot0Configs();
        HoodMotorPIDConfig.kP = Constants.LauncherConstants.HOOD_MOTOR_KP;
        HoodMotorPIDConfig.kI = Constants.LauncherConstants.HOOD_MOTOR_KI;
        HoodMotorPIDConfig.kD = Constants.LauncherConstants.HOOD_MOTOR_KD;

        mHoodMotor.getConfigurator().apply(mHoodMotorConfig);
        mHoodMotor.getConfigurator().apply(HoodMotorPIDConfig);
        mHoodMotor.getConfigurator().apply(mHoodMotorSoftLimitConfig);

        //mHoodMotor.configure(mHoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Transfer Setup

        CurrentLimitsConfigs transferCurrentLimitConfigs = new CurrentLimitsConfigs();
        flywheelCurrentLimitConfigs.StatorCurrentLimit = Constants.LauncherConstants.TRANSFER_STATOR_CURRENT_LIMIT;
        flywheelCurrentLimitConfigs.SupplyCurrentLimit = Constants.LauncherConstants.TRANSFER_SUPPLY_CURRENT_LIMIT;

        Slot0Configs transferPIDConfig = new Slot0Configs();
        transferPIDConfig.kP = Constants.LauncherConstants.TRANSFER_KP;
        transferPIDConfig.kI = Constants.LauncherConstants.TRANSFER_KI;
        transferPIDConfig.kD = Constants.LauncherConstants.TRANSFER_KD;
        transferPIDConfig.kS = Constants.LauncherConstants.TRANSFER_KS;
        transferPIDConfig.kV = Constants.LauncherConstants.TRANSFER_KV;

        MotorOutputConfigs transferOutputConfigs = new MotorOutputConfigs();
        transferOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfiguration transferConfig = new TalonFXConfiguration(); 
        transferConfig.CurrentLimits = transferCurrentLimitConfigs;
        transferConfig.Slot0 = transferPIDConfig;
        transferConfig.MotorOutput = transferOutputConfigs;
        mTransfer.getConfigurator().apply(transferConfig);

        
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
        SmartDashboard.putNumber("Launcher/ShotTuning/Angle", 15.0);
        SmartDashboard.putNumber("Launcher/ShotTuning/FlywheelRPM", 2800);
        SmartDashboard.putNumber("Launcher/ShotTuning/FeedSpeed", 8.0);
        SmartDashboard.putBoolean("Launcher/ShotTuning/PreciseShot", true);
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
        mFlywheelRequest = new VelocityVoltage(mTargetSpeed); //Saving RAM by only instantiating this on a change.
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
        return mFlywheelLeader.getVelocity().getValue();
    }

    /**
     * Shut down everything
     */
    public void stop(){
        mFlywheelLeader.stopMotor();
        //#jacobtodo
        mHopper.stopMotor();
        mTransfer.stopMotor();
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
                mHoodMotor.setControl(new PositionVoltage(mTargetAngle));
                //mHoodMotor.getClosedLoopController().setSetpoint(mTargetAngle.in(Degrees), ControlType.kPosition);
                mFlywheelLeader.setControl(mFlywheelRequest); 
                mHopper.stopMotor();
                mTransfer.stopMotor();
                if(isAtSpeed())
                {
                    mFlywheelEstimator.addLast(estimatekF());
                    if(mFlywheelEstimator.size() >= Constants.LauncherConstants.SAMPLES_TO_AVERAGE && (mEnableLaunch || getSmartEnableLaunch()))
                    {
                        savekF();
                        mLauncherState = LauncherStates.kOpenLoop;
                        mStartLaunchingTime = Timer.getFPGATimestamp();
                    }
                }

                break;
            case kOpenLoop:
                mHoodMotor.setControl(new PositionVoltage(mTargetAngle));
                //mHoodMotor.getClosedLoopController().setSetpoint(mTargetAngle.in(Degrees), ControlType.kPosition);
                //mFlywheelLeader.setControl(mFlywheelVoltageOut);
                mFlywheelLeader.setControl(mFlywheelRequest); 
                if(!isAtSpeed() && mFlywheelLeader.getVelocity().getValueAsDouble() > mTargetSpeed.in(RotationsPerSecond)){
                    estimatekF();
                    savekF();
                }
                if(!mEnableLaunch && !getSmartEnableLaunch()){
                    mLauncherState = LauncherStates.kClosedLoop;
                }
                mHopper.setControl(new VoltageOut(mFeedSpeed));
                mTransfer.setControl(mTransferMoveRequest);
                //mTransfer.setControl(new VoltageOut(mFeedSpeed));
                break;
            case kCleaningMode:
                 mHoodMotor.setControl(new PositionVoltage(Constants.LauncherConstants.HOOD_HOME_POSITION));
                 //mHoodMotor.getClosedLoopController().setSetpoint(Constants.LauncherConstants.HOOD_HOME_POSITION.in(Degrees), ControlType.kPosition);
                 mTransfer.setVoltage(3.0);
                //DO Nothing for cleaning mode, managed externally
            break;
            case kZeroHood:
                mHoodMotor.set(-.3);
                mFlywheelLeader.set(0);
                mHopper.stopMotor();
                mTransfer.stopMotor();
                if(Math.abs(mHoodMotor.getSupplyCurrent().getValue().in(Amps)) > 2 && Math.abs(mHoodMotor.getVelocity().getValue().in(DegreesPerSecond)) < 0.1)
                {
                    mHoodMotor.setPosition(0.0);
                    mHoodMotor.set(0);
                    mLauncherState = LauncherStates.kIdle;
                    mHoodMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
                    mHoodMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
                    mHoodMotor.getConfigurator().apply(mHoodMotorSoftLimitConfig);
                }
            break;
            case kTestHood:
                mHoodMotor.setControl(new PositionVoltage(mTargetAngle));
                mFlywheelLeader.stopMotor();
                mHopper.stopMotor();
                mTransfer.stopMotor();
                break;
            case kIdle:  //Intentionally no-break after kIdle, we want kIdle to be effectively the default
            default:
                mHoodMotor.setControl(new PositionVoltage(Constants.LauncherConstants.HOOD_HOME_POSITION));
                mFlywheelLeader.stopMotor();
                mHopper.stopMotor();
                mTransfer.stopMotor();
                break;
            
        }

        SmartDashboard.putNumber("Launcher/RPM", getFlywheelVelocity().in(RPM));
        SmartDashboard.putNumber("Launcher/Angle", mHoodMotor.getPosition().getValue().in(Degrees));
        SmartDashboard.putString("Launcher/State", mLauncherState.name());
        SmartDashboard.putNumber("Launcher/Target RPM", mTargetSpeed.magnitude());
        SmartDashboard.putNumber("Launcher/TargetAngle", mTargetAngle.in(Degrees));
        SmartDashboard.putNumber("Launcher/kF", mKf.magnitude());
        SmartDashboard.putNumber("Launcher/kF Samples", mFlywheelEstimator.size());
        SmartDashboard.putNumber("Launcher/HoodMotorAmps", mHoodMotor.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("Launcher/Sim/SimSpeed", mFlywheelSimulation.getAngularVelocityRPM());
        SmartDashboard.putNumber("Launcher/Sim/SimAmps", mFlywheelSimulation.getCurrentDrawAmps());
        SmartDashboard.putNumber("Launcher/Sim/SimInputVolts", mFlywheelSimulation.getInputVoltage());
        SmartDashboard.putNumber("Launcher/Sim/SimKrakenMotorVolts", mFlywheelLeaderSim.getMotorVoltage());
        SmartDashboard.putNumber("Launcher/TransferRPM", mTransfer.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Launcher/TransferCurrent", mTransfer.getStatorCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("Launcher/HopperAmps", mHopper.getStatorCurrent().getValue().in(Amps));

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

    public Command prepareDashboardShot(){
        return new Command(){

            @Override
            public Set<Subsystem> getRequirements() {
                HashSet<Subsystem> reqs = new HashSet<>();
                reqs.add(RobotContainer.launcher);
                return reqs;
            }

            @Override
            public void execute(){
                shoot(RPM.of(SmartDashboard.getNumber("Launcher/ShotTuning/FlywheelRPM", 1500)), Degrees.of(SmartDashboard.getNumber("Launcher/ShotTuning/Angle", 25)),  Volt.of(SmartDashboard.getNumber("Launcher/ShotTuning/FeedSpeed", 8.0)));
            };
        };
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
        mHoodMotorSoftLimitConfig.ForwardSoftLimitEnable = false;
        mHoodMotorSoftLimitConfig.ReverseSoftLimitEnable = false;
        mHoodMotor.getConfigurator().apply(mHoodMotorSoftLimitConfig);
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

        public Command launcherAimForFeed(){
        return new Command(){

            @Override
            public Set<Subsystem> getRequirements() {
                HashSet<Subsystem> reqs = new HashSet<>();
                reqs.add(RobotContainer.launcher);
                return reqs;
            }

            @Override
            public void execute(){
                shoot(VisionShotGenerator.GetGoalShotForDistance(mDrivebase.getDistanceToFeed()));
            };
        };
    }

    public void testHood15(){
        mTargetAngle = Degrees.of(15);
        mLauncherState = LauncherStates.kTestHood;
    }

    public void testHood30(){
        mTargetAngle = Degrees.of(30);
        mLauncherState = LauncherStates.kTestHood;
    }

    public void testHood45(){
        mTargetAngle = Degrees.of(45);
        mLauncherState = LauncherStates.kTestHood;
    }

    public double getLaunchingTime(){
        if(mLauncherState != LauncherStates.kOpenLoop){
            return 0.0;
        }
        else{
            return Timer.getFPGATimestamp() - mStartLaunchingTime;
        }
    }
}
