package frc.robot;

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.configuration2.FileBasedConfiguration;
import org.apache.commons.configuration2.PropertiesConfiguration;
import org.apache.commons.configuration2.builder.FileBasedConfigurationBuilder;
import org.apache.commons.configuration2.builder.fluent.Parameters;
import org.apache.commons.configuration2.ex.ConfigurationException;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotPreferences {

    private enum Preferences{
        IntakeExtendOnlyInUse(false),
        ShooterAngleOffset(0.0),
        IntakeUseAsAgitator(false);

        Object mDefaultVal;
        private Preferences(Object defaultVal){
            mDefaultVal = defaultVal;
        }
        
    }

    private static Parameters parameters = new Parameters();
    private static Map<String, Object> propertiesCache = new HashMap<>();

    private static FileBasedConfigurationBuilder<FileBasedConfiguration> builder = 
        new FileBasedConfigurationBuilder<FileBasedConfiguration>(PropertiesConfiguration.class)
            .configure(parameters.fileBased().setFileName("26robot.properties").setBasePath(Filesystem.getDeployDirectory().getAbsolutePath()));

            /**
             * This loads everything to store it in the cache
             * @param offset
             */
    public static void LoadOnBoot(){
        builder.setAutoSave(true);
        for(Preferences preference : Preferences.values()){
            LoadPreference(preference);
        }
        String outputString = new String();
        outputString +="Preferences Loaded!{\n";
        for(String preferenceName: propertiesCache.keySet()){
            outputString += (preferenceName + ":" + propertiesCache.get(preferenceName) + ",\n");
        }
        outputString += "\n}\n";
        System.out.print(outputString);
        try {
            builder.save();
        } catch (ConfigurationException e) {
            System.out.println("ROBOT PREFERENCES : Failed to load preferences on boot!");// TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private static Object LoadPreference(Preferences preference){
        if(propertiesCache.get(preference.name()) == null){
            try {
                Object prefVal = builder.getConfiguration().getProperty(preference.name());
                if(prefVal != null){
                    propertiesCache.put(preference.name(), prefVal);
                    return prefVal;
                }
                else{
                    SavePreference(preference, preference.mDefaultVal);
                    return preference.mDefaultVal;
                }
            } catch (ConfigurationException e) {
                System.out.println("Failed to load preference " + preference.name());
                e.printStackTrace();
                SavePreference(preference, preference.mDefaultVal);
                return preference.mDefaultVal;
               
            }
        }
        else{
            return propertiesCache.get(preference.name());
        }
    }

    private static void SavePreference(Preferences preference, Object valueToSet){
        try {
            propertiesCache.put(preference.name(), valueToSet.toString());
            builder.getConfiguration().setProperty(preference.name(), valueToSet);
        } catch (ConfigurationException e) {
            System.out.println("Failed to save preference " + preference.name());
            e.printStackTrace();
        }
    }


    public static boolean GetIntakeExtendOnlyInUse(){
        return Boolean.parseBoolean((String) LoadPreference(Preferences.IntakeExtendOnlyInUse));
    }

    public static void SetIntakeExtendOnlyInUse(boolean valueToSet){
        SavePreference(Preferences.IntakeExtendOnlyInUse, valueToSet);
    }

    public static double GetShooterAngleOffset(){
        return Double.parseDouble((String) LoadPreference(Preferences.ShooterAngleOffset));
    }

    public static void SetShooterAngleOffset(double offset){
        SavePreference(Preferences.ShooterAngleOffset, offset);
    }

    public static boolean GetIntakeUseAsAgitator(){
        return Boolean.parseBoolean((String) LoadPreference(Preferences.IntakeUseAsAgitator));
    }

    public static void SetIntakeUseAsAgitator(boolean use){
        SavePreference(Preferences.IntakeUseAsAgitator, use);
    }

}
