package frc.robot.subsystems.Handle.Templates;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotorRollerBase extends TalonFX {

    public TalonFXConfiguration config;
    public final String name;
        
    public MotorRollerBase(int id, String name, double gearRatio) {
        super(id);
        this.name = name;

        config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio));
        this.config.MotionMagic.MotionMagicCruiseVelocity = 10d;         
        this.getConfigurator().apply(config);       

    }

    /** Set a param to null if you don't want to touch it. It'll handle it. */
    public void setMovement(Double cruiseVel, Double accel, Double jerk) {
        if (cruiseVel != null) this.config.MotionMagic.MotionMagicCruiseVelocity = cruiseVel;
        if (accel != null) this.config.MotionMagic.MotionMagicAcceleration = accel;
        if (jerk != null) this.config.MotionMagic.MotionMagicJerk = jerk;
    }


    public void follow(MotorRollerBase base) {
        this.config.MotionMagic = base.config.MotionMagic; //dubious
    }

    public void reapplyConfigurator() {
        this.getConfigurator().apply(config);
    }

    public void setRPM(double d) {

    }
}
