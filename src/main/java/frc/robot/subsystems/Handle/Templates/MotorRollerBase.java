package frc.robot.subsystems.Handle.Templates;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotorRollerBase extends TalonFX {

    public VelocityVoltage rpmController;
    public TalonFXConfiguration config;
    
    public MotorRollerBase(int id, double gearRatio) {
        super(id);
        this.rpmController = new VelocityVoltage(0);
        config = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio)); 
        this.getConfigurator().apply(config);       
    }

    public void follow(MotorRollerBase base) {
        this.rpmController = base.rpmController;
    }

    public void reapplyConfigurator() {
        this.getConfigurator().apply(config);
    }

    public void setRPM(double d) {
        this.setControl(rpmController.withVelocity(d));
    }
}
