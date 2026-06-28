package frc.robot.subsystems.Handle.Templates;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class MotorPivotBase extends TalonFX {

    public MotionMagicVoltage posController;
    public TalonFXConfiguration config;
    public final String name;
    public Follower follower = null;

    //try to not set the following of motors in the motor
        
    public MotorPivotBase(int id, String name, double gearRatio) {
        super(id);
        this.posController = new MotionMagicVoltage(0);
        this.name = name;
        

        config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio)); 
        this.getConfigurator().apply(config);       
    }

    public void follow(MotorPivotBase base, MotorAlignmentValue value) {
        follower = new Follower(base.getDeviceID(), value);
        this.setControl(follower);
    }

    public void reapplyConfigurator() {
        this.getConfigurator().apply(config);
    }

    public void setPos(double d) {
        if (follower != null) throw new RuntimeException("Don't set the follower " + name + " to move!");
        this.setControl(posController.withPosition(d));
    }
}
