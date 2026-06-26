package frc.robot.subsystems.Handle.Templates;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotorPivotBase extends TalonFX {

    public MotionMagicVoltage posController;
    public TalonFXConfiguration config;
    public final String name;
        
    public MotorPivotBase(int id, String name, double gearRatio) {
        super(id);
        this.posController = new MotionMagicVoltage(0);
        this.name = name;

        config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio)); 
        this.getConfigurator().apply(config);       
    }

    public void follow(MotorPivotBase base) {
        this.posController = base.posController;
    }

    public void reapplyConfigurator() {
        this.getConfigurator().apply(config);
    }

    public void setPos(double d) {
        this.setControl(posController.withPosition(d));
    }
}
