package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MatchTimer extends SubsystemBase{
    private static final double TIME_BEFORE_OFFICIAL_ACTIVE = 2.0; //seconds
    private static final double  TIME_AFTER_OFFICIAL_ACTIVE = 1.0;
    private static final double CLIMB_TIME = 15.0;
    private static final double AUTO_WINNER_LOAD_TIME = 8.0;
    private static final double MAX_DRIFT_THRESHOLD = 3.0; // Resync if drift exceeds 3 seconds

    public enum TeleopMatchTimeframe{
        NOT_TELEOP(-1,-1),
        TRANSITION_SHIFT(140, 130),
        SHIFT_1(130, 105),
        SHIFT_2(105,80),
        SHIFT_3(80, 55),
        SHIFT_4(55, 30),
        END_GAME(30, 0)
        ;
        public double teleopTimeLeftAtStart;
        public double teleopTimeLeftAtEnd;
        private TeleopMatchTimeframe(double teleopTimeLeftStart, double teleopTimeLeftEnd){
            teleopTimeLeftAtStart = teleopTimeLeftStart;
            teleopTimeLeftAtEnd = teleopTimeLeftEnd;
        }
    }

    public class HubState{
        public double mTimeLeftBeforeChange;
        public boolean mIsActive;
        public TeleopMatchTimeframe matchTimeframe;
        public HubState(double mTimeLeftBeforeChange, boolean mIsActive) {
            this.mTimeLeftBeforeChange = mTimeLeftBeforeChange;
            this.mIsActive = mIsActive;
            this.matchTimeframe = matchTimeframe;
        }
        public double getTimeLeftBeforeChange() {
            return mTimeLeftBeforeChange;
        }
        public boolean isActive() {
            return mIsActive;
        }

    }

    double teleopTimeLeft;
    double lastUpdateTimeStamp;
    double currentTime;
    boolean running = false;
    boolean wonInAuto = false;
    boolean loadedWonInAuto = false;

    NetworkTableInstance ntinst;
    NetworkTable fmsInfo;
    NetworkTable robotTable;


    public MatchTimer(){
        teleopTimeLeft = 140;
        lastUpdateTimeStamp = 0;
        running = false;
        wonInAuto = false;
        loadedWonInAuto = false;
        ntinst = NetworkTableInstance.getDefault();
        fmsInfo = ntinst.getTable("FMSInfo");
        robotTable = ntinst.getTable("Robot");

    }

    public synchronized void startTimer(){
        teleopTimeLeft = 140.0;
        lastUpdateTimeStamp = Timer.getFPGATimestamp();
        running = true;
    }

    public synchronized boolean isTeleopTimerExpired(){
        return teleopTimeLeft<0;
    }

    public synchronized void stopTimer(){
        teleopTimeLeft = -1;
        lastUpdateTimeStamp = Timer.getFPGATimestamp();
        running = false;
    }

    public boolean isRunning(){
        return running;
    }

    public HubState getHubState(){
        if(!running)
        {
            if(DriverStation.isAutonomousEnabled()){
                return new HubState(DriverStation.getMatchTime(), true);
            }
            return new HubState(-1, false);
        }
        else{
            if(teleopTimeLeft > TeleopMatchTimeframe.TRANSITION_SHIFT.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE)
            {
                return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.TRANSITION_SHIFT.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE), true);
            }
            else if (wonInAuto)
            {
                //We were ahead in fuel in auto
                if(teleopTimeLeft > TeleopMatchTimeframe.SHIFT_1.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_1.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE), false);
                }else if ( teleopTimeLeft > TeleopMatchTimeframe.SHIFT_2.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_2.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE), true);
                }else if (teleopTimeLeft > TeleopMatchTimeframe.SHIFT_3.teleopTimeLeftAtEnd +TIME_BEFORE_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_3.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE), false);
                }else{
                    return new HubState(teleopTimeLeft, true);
                }
            }
            else{
                //We were behind in fuel in auto
                if(teleopTimeLeft > TeleopMatchTimeframe.SHIFT_1.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_1.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE), true);
                }else if ( teleopTimeLeft > TeleopMatchTimeframe.SHIFT_2.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_2.teleopTimeLeftAtEnd +TIME_BEFORE_OFFICIAL_ACTIVE), false);
                }else if (teleopTimeLeft > TeleopMatchTimeframe.SHIFT_3.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_3.teleopTimeLeftAtEnd - TIME_AFTER_OFFICIAL_ACTIVE), true);
                }else if (teleopTimeLeft > TeleopMatchTimeframe.SHIFT_4.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE){
                    return new HubState(teleopTimeLeft - (TeleopMatchTimeframe.SHIFT_4.teleopTimeLeftAtEnd + TIME_BEFORE_OFFICIAL_ACTIVE), false);
                }else{
                    return new HubState(teleopTimeLeft, true);
                }
            }
        }
    }

    public Boolean getWonInAuto(){
        return wonInAuto;
    }

    private Boolean calculateWonInAuto(){
        if(!DriverStation.getGameSpecificMessage().isEmpty()){
            return DriverStation.getGameSpecificMessage().toLowerCase().charAt(0) == 
                DriverStation.getAlliance().orElse(Alliance.Red).name().toLowerCase().charAt(0);
        }
        return false;
    }

    @Override
    public void periodic(){
        if(running && teleopTimeLeft > 0){
            // Check for large drift only during real matches (FMS attached)
            if(DriverStation.isTeleop() && DriverStation.isFMSAttached()){
                double fmsMatchTime = DriverStation.getMatchTime();
                if(fmsMatchTime >= 0 && fmsMatchTime <= 135){
                    double drift = Math.abs(teleopTimeLeft - fmsMatchTime);
                    if(drift > MAX_DRIFT_THRESHOLD){
                        // Large drift detected - resync to FMS time
                        System.out.println("MatchTimer: Large drift detected (" +
                            String.format("%.2f", drift) + "s) - resyncing to FMS time");
                        teleopTimeLeft = fmsMatchTime;
                        lastUpdateTimeStamp = Timer.getFPGATimestamp();
                    } else {
                        // Normal operation - use precise FPGA timing
                        currentTime = Timer.getFPGATimestamp();
                        teleopTimeLeft -= (currentTime - lastUpdateTimeStamp);
                        lastUpdateTimeStamp = currentTime;
                    }
                } else {
                    // FMS time unavailable - fallback to FPGA timing
                    currentTime = Timer.getFPGATimestamp();
                    teleopTimeLeft -= (currentTime - lastUpdateTimeStamp);
                    lastUpdateTimeStamp = currentTime;
                }
            } else {
                // Not in real match or not in teleop - use FPGA timing only
                currentTime = Timer.getFPGATimestamp();
                teleopTimeLeft -= (currentTime - lastUpdateTimeStamp);
                lastUpdateTimeStamp = currentTime;
            }
        }
        if(!loadedWonInAuto && teleopTimeLeft < 140 - AUTO_WINNER_LOAD_TIME )
        {
            wonInAuto = calculateWonInAuto();
            loadedWonInAuto = true;
        }

        writeToNetworkTables();
    }

    public void writeToNetworkTables(){
        if(DriverStation.isDisabled()){
            robotTable.putValue("MatchState", NetworkTableValue.makeString("pre-match"));
        }else if (DriverStation.isAutonomous()){
            robotTable.putValue("MatchState", NetworkTableValue.makeString("auto"));
            robotTable.putValue("Timer/GoalActive", NetworkTableValue.makeBoolean(true));
            robotTable.putValue("Timer/TimeRemaining", NetworkTableValue.makeDouble(DriverStation.getMatchTime()));
        } else if (DriverStation.isTeleop() && teleopTimeLeft < CLIMB_TIME){
            robotTable.putValue("MatchState", NetworkTableValue.makeString("endgame"));
        } else {
            robotTable.putValue("MatchState", NetworkTableValue.makeString("teleop"));
        }

        if(DriverStation.isTeleop()){
            HubState teleopHubState = getHubState();
            robotTable.putValue("Timer/GoalActive", NetworkTableValue.makeBoolean(teleopHubState.mIsActive));
            robotTable.putValue("Timer/TimeRemaining", NetworkTableValue.makeDouble(teleopHubState.mTimeLeftBeforeChange));
        }

    }

    public Trigger isHubEnabled(){
        return new Trigger(() ->getHubState().mIsActive);
    }

    public Trigger isHubAboutToEnable(){
        return new Trigger(() -> !getHubState().mIsActive && getHubState().mTimeLeftBeforeChange < 5.0);
    }

        public Trigger isHubAboutToDisable(){
        return new Trigger(() -> getHubState().mIsActive && getHubState().mTimeLeftBeforeChange < 3 && !(teleopTimeLeft > 110.0 & !getWonInAuto()));
    }

    

}
