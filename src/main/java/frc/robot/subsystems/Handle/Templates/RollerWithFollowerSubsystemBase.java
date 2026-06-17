package frc.robot.subsystems.Handle.Templates;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class RollerWithFollowerSubsystemBase<T extends Enum<T>> extends SubsystemHandle<T> {
    public TalonFX motor;
    public TalonFX follower;
    private String internalName;
    public VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    public double RPM = 0;
    public double motorGearRatio;
    public double followerGearRatio;
    
        public RollerWithFollowerSubsystemBase(T e, int rollerId, int followerId, MotorAlignmentValue alignment, String subsystemName, double motorGearRatio, double followerGearRatio, boolean invertFollower) {
            super(e);
            motor = new TalonFX(rollerId, Constants.krakenBus);
            follower = new TalonFX(followerId, Constants.krakenBus);
            
            this.motorGearRatio = motorGearRatio;
            this.followerGearRatio = followerGearRatio;
    
    
         //keep this in case we want to reduce parameters   
         //   String[] fullName = this.getClass().getName().split(".");
         //   internalName = fullName[fullName.length - 1];
        // double rollerRatio;
        // double followerRatio;
    
        // try {
        //     rollerRatio = (Double) Constants.class.getField(internalName + "Roller").get(null);
        // } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
        //     throw new RuntimeException("Cannot find: " + internalName + "Roller" + " within Constants.java! Cause: " + e1.getMessage());
        // }
    
        // try {
        //     followerRatio = (Double) Constants.class.getField(internalName + "Follower").get(null);
        // } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
        //     throw new RuntimeException("Cannot find: " + internalName + "Roller" + " within Constants.java! Cause: " + e1.getMessage());
        // }
    
         internalName = subsystemName;
    
    
    
        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(motorGearRatio));
        TalonFXConfiguration followerConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(followerGearRatio));
    
        setMotorConfiguration(motorConfig);
        setFollowerConfiguration(followerConfig);
    
        if (invertFollower)
            followerConfig.MotorOutput.Inverted = (followerConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;        
    

        motor.getConfigurator().apply(motorConfig);
        follower.getConfigurator().apply(followerConfig);
    
        motor.setControl(velocityVoltage.withVelocity(RPM));
        follower.setControl(velocityVoltage.withVelocity(RPM));
        }
    
    /**<p> An alternative way of creating a subsystem with dramatically less parameters. May be dangerous. </p>
     * <p> Employs reflection to retrieve data from Constants.java.</p>
     * <p> Subsystem name originates from the name of the file containing the subsystem, which may include the
     *     phrase "Subsystem". Case sensitive.</p>
     * <p> . </p>
     * <p> Required parameters within Constants.java: </p>
     * <p> [subsystem name]RollerGearRatio </p>
     * <p> [subsystem name]RollerId </p>
     * <p> [subsystem name]FollowerGearRatio </p>
     * <p> [subsystem name]FollowerId </p>
     */
    public RollerWithFollowerSubsystemBase(T e, boolean invertFollower) {
        super(e);
        



        String[] fullName = this.getClass().getName().split(".");
        internalName = fullName[fullName.length - 1];

        double motorGearRatio;
        double followerGearRatio;

        int motorId;
        int followerId;

        try {
            motorGearRatio = (Double) Constants.class.getField(internalName + "RollerGearRatio").get(null);
        } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
            throw new RuntimeException("Cannot find: " + internalName + "RollerGearRatio" + " within Constants.java! Cause: " + e1.getMessage());
        }
        try {
            followerGearRatio = (Double) Constants.class.getField(internalName + "FollowerGearRatio").get(null);
        } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
            throw new RuntimeException("Cannot find: " + internalName + "FollowerGearRatio" + " within Constants.java! Cause: " + e1.getMessage());
        }

        try {
            motorId = (Integer) Constants.class.getField(internalName + "RollerId").get(null);
        } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
            throw new RuntimeException("Cannot find: " + internalName + "RollerId" + " within Constants.java! Cause: " + e1.getMessage());
        }

        try {
            followerId = (Integer) Constants.class.getField(internalName + "FollowerId").get(null);
        } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e1) {
            throw new RuntimeException("Cannot find: " + internalName + "FollowerId" + " within Constants.java! Cause: " + e1.getMessage());
        }
        


        motor = new TalonFX(motorId, Constants.krakenBus);
        follower = new TalonFX(followerId, Constants.krakenBus);
        this.motorGearRatio = motorGearRatio;
        this.followerGearRatio = followerGearRatio;


        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(motorGearRatio));
        TalonFXConfiguration followerConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(followerGearRatio));

        setMotorConfiguration(motorConfig);
        setFollowerConfiguration(followerConfig);

        if (invertFollower)
            followerConfig.MotorOutput.Inverted = (followerConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;



        motor.getConfigurator().apply(motorConfig);
        follower.getConfigurator().apply(followerConfig);

        motor.setControl(velocityVoltage.withVelocity(RPM));
        follower.setControl(velocityVoltage.withVelocity(invertFollower ? -RPM : RPM));
    
    }

    @Overridable
    public void setMotorConfiguration(TalonFXConfiguration config) {}

    @Overridable
    public void setFollowerConfiguration(TalonFXConfiguration config) {}

    @Overridable
    public void logging() {
        log("CurrentState", currentState);
        log("WantedState", wantedState);

        log("RollerRPS", motor.getVelocity().getValueAsDouble());
        log("FollowerRPS", follower.getVelocity().getValueAsDouble());

        log("rollerTemp", motor.getDeviceTemp().getValueAsDouble());
        log("followerTemp",  follower.getDeviceTemp().getValueAsDouble());

        log("rollerCurrent", motor.getStatorCurrent().getValueAsDouble());
        log("followerCurrent", follower.getStatorCurrent().getValueAsDouble());

        log("rollerVoltage", motor.getMotorVoltage().getValueAsDouble());
        log("followerVoltage", follower.getMotorVoltage().getValueAsDouble());      
        
    }

    public final void log(String name, Object data) {
        Logger.recordOutput(internalName + "/" + name, data.toString());
    }
}