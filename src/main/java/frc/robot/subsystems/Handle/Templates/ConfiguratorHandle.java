package frc.robot.subsystems.Handle.Templates;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;


//FIRST PROTOTYPE COMPLETE

/** not sure how to go about the template. There isn't much to do here, short of printing everything (not a good idea) */
public abstract class ConfiguratorHandle {

    /** Treat this as a way to map a piece of information - in this case, the name - to some info, in this case the talonfx. */
    public Map<String, TalonFXConfiguration> motors = new HashMap<String, TalonFXConfiguration>();

    /** These serve a dual purpose - to provide a getter, and demonstrate how the hashmap works. */
    public TalonFXConfiguration getMotor(String name) {
        return motors.get(name);
    }

    /** These serve a dual purpose - to provide a setter, and demonstrate how the hashmap works. */
    public TalonFXConfiguration setMotor(String name, TalonFXConfiguration talonfx) {

        return motors.put(name, talonfx);
    }

    /** @Param values Goes in order of kP, kI, kD, kV, kA, kS, kG. Place null to ignore, or just end. */
    public void setPID(String name, Integer... values) {
        TalonFXConfiguration talonfx = getMotor(name);

        if (values.length > 0 && values[0] != null) talonfx.Slot0.kP = values[0];
        if (values.length > 1 && values[1] != null) talonfx.Slot0.kI = values[1];
        if (values.length > 2 && values[2] != null) talonfx.Slot0.kD = values[2];
        if (values.length > 3 && values[3] != null) talonfx.Slot0.kV = values[3];
        if (values.length > 4 && values[4] != null) talonfx.Slot0.kA = values[4];
        if (values.length > 5 && values[5] != null) talonfx.Slot0.kS = values[5];
        if (values.length > 6 && values[6] != null) talonfx.Slot0.kG = values[6];
    }

    public void setSupplyAndStatorLimits(String name, Double supplylimit, Double statorlimit) {
        TalonFXConfiguration talonfx = getMotor(name);

        if (supplylimit != null) {
            talonfx.CurrentLimits.SupplyCurrentLimit = supplylimit;
            talonfx.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        if (statorlimit != null) {
            talonfx.CurrentLimits.StatorCurrentLimit = statorlimit;
            talonfx.CurrentLimits.StatorCurrentLimitEnable = true;
        }
    }

    public void setBreakAndInversion(String name, NeutralModeValue breakmode, InvertedValue inverted) {
        TalonFXConfiguration talonfx = getMotor(name);

        if (breakmode != null) talonfx.MotorOutput.NeutralMode = breakmode;
        if (inverted != null) talonfx.MotorOutput.Inverted = inverted;
    }

    public void setMotionMagic(String name, Double accel, Double jerk, Double cruisevelocity) {
        TalonFXConfiguration talonfx = getMotor(name);

        if (accel != null) talonfx.MotionMagic.MotionMagicAcceleration = accel;
        if (jerk != null)talonfx.MotionMagic.MotionMagicJerk = jerk;
        if (cruisevelocity != null) talonfx.MotionMagic.MotionMagicCruiseVelocity = cruisevelocity;
        
    }

    //im not putting up these signs for no reason

    /** WARNING - DANGEROUS REFLECTION - 
    *   CALL OR CHANGE AT YOUR SYSTEM'S RISK
    * 
    * @param subsystemName The subsystem to create motors from.
    * @throws IllegalAccessException 
    * @throws IllegalArgumentException */
    public void autoSetMotors(String subsystemName) throws IllegalArgumentException, IllegalAccessException {

    //Iterate fields.    
        for (Field field : Constants.class.getFields()) {

            //Check if field is applicable.
            if (!field.canAccess(null)) continue;
            if (!field.getName().endsWith("ID")) continue;
            if (!field.getName().startsWith(subsystemName)) continue;
            if (!field.getType().equals(Integer.class)) continue;
            

            //Place the motor. Most dangerous section, does heavy lifting.

            String motorname = field.getName().split("ID")[0].split(subsystemName)[1];
            double ratio = 1;

            try {
                ratio = Constants.class.getField(motorname + "GearRatio").getDouble(null);
            } catch (Exception e) {}

            motors.put(
            motorname,
            new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ratio))
            );
        }
    }
 













//dunno what to do with this thing - delete if not needed

/*
 */
}

