package frc.robot.subsystems.Handle.Templates;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.subsystems.Handle.SubsystemHandle;

public abstract class RollerWithFollowerBase<T extends Enum<T>> extends SubsystemHandle<T> {
    public TalonFX motor;
    public TalonFX follower;
    private String internalName;
    public VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    public double RPM = 0;
    public double motorGearRatio;
    public double followerGearRatio;

    public RollerWithFollowerBase(T e, int rollerId, int followerId, MotorAlignmentValue alignment, String subsystemName, double motorGearRatio, double followerGearRatio) {
        super(e);
        motor = new TalonFX(rollerId, Constants.krakenBus);
        follower = new TalonFX(followerId, Constants.krakenBus);
        
        this.motorGearRatio = motorGearRatio;
        this.followerGearRatio = followerGearRatio;


     //keep this in case we want to reduce parameters   
     //   String[] fullName = this.getClass().getName().split(".");
     //   internalName = fullName[fullName.length - 1];
     internalName = subsystemName;



    TalonFXConfiguration motorConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(motorGearRatio));
    TalonFXConfiguration followerConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(followerGearRatio));

    setMotorConfiguration(motorConfig);
    setFollowerConfiguration(followerConfig);

    motor.getConfigurator().apply(motorConfig);
    follower.getConfigurator().apply(followerConfig);

    motor.setControl(velocityVoltage.withVelocity(RPM));
    follower.setControl(velocityVoltage.withVelocity(RPM));
    }

    @Overridable
    public void setMotorConfiguration(TalonFXConfiguration config) {}

    @Overridable
    public void setFollowerConfiguration(TalonFXConfiguration config) {}

    @Overridable
    public void logging() {
        log("CurrentState", currentState);
        log("WantedState", wantedState);

        log("RollerRPS", 0);
        log("FollowerRPS", 0);

        log("rollerTemp", 0);
        log("followerTemp", 0);

        log("rollerCurrent", 0);
        log("followerCurrent", 0);

        log("rollerVoltage", 0);
        log("followerVoltage", 0);      
        
    }


    public final void log(String name, Object data) {
        Logger.recordOutput(internalName + "/" + name, data.toString());
    }
}


