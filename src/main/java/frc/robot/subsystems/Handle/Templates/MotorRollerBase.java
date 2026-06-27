package frc.robot.subsystems.Handle.Templates;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class MotorRollerBase extends TalonFX {

    public TalonFXConfiguration config;
    public final String name;
    public Follower follower = null;
        
    public MotorRollerBase(int id, String name, double gearRatio) {
        super(id);
        this.name = name;

        config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio));
        this.config.MotionMagic.MotionMagicCruiseVelocity = 10d;         
        this.getConfigurator().apply(config);       

    }

    /** Set a param to null if you don't want to touch it. It'll handle it. */
    public void setMovement(Double cruiseVel, Double accel, Double jerk) {
        if (follower != null) throw new RuntimeException("Don't set the follower " + name + " to move!");
        if (cruiseVel != null) this.config.MotionMagic.MotionMagicCruiseVelocity = cruiseVel;
        if (accel != null) this.config.MotionMagic.MotionMagicAcceleration = accel;
        if (jerk != null) this.config.MotionMagic.MotionMagicJerk = jerk;
    }


    public void follow(MotorRollerBase base, MotorAlignmentValue value) {
        follower = new Follower(base.getDeviceID(), value);
        this.setControl(follower);
    }

    public void reapplyConfigurator() {
        this.getConfigurator().apply(config);
    }
}
