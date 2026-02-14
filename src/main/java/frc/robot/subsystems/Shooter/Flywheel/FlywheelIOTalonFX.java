package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;

public class FlywheelIOTalonFX implements FlywheelIO{
    public TalonFX FlywheelLeftMotor;
    public TalonFX FlywheelRightMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVelocityVoltage FlywheelRightMotorRequest;
    public FlywheelIOTalonFX() {
         shooterConfigs = new ShooterConfigurator();
        FlywheelLeftMotor = new TalonFX(Constants.flyWheelRightID,Constants.krakenBus);
        FlywheelRightMotor = new TalonFX(Constants.flyWheelLeftID,Constants.krakenBus);

        FlywheelRightMotor.getConfigurator().apply(shooterConfigs.flyWheelRightConfig);
    }
    @Override
      public void updateInputs(FlywheelIOInputs inputs){
        inputs.FlywheelLeftRPS = FlywheelLeftMotor.getVelocity().getValueAsDouble();
        inputs.FlywheelRightRPS = FlywheelRightMotor.getVelocity().getValueAsDouble();
        inputs.FlywheelLeftMotorTemp = FlywheelLeftMotor.getDeviceTemp().getValueAsDouble();
        inputs.FlywheelRightMotorTemp = FlywheelRightMotor.getDeviceTemp().getValueAsDouble();
    }
    
    public void setFlywheelVelocity(FlywheelStates state){
        FlywheelRightMotorRequest.Velocity = state.FlywheelRightRPS;
        FlywheelLeftMotor.setControl(new Follower(Constants.flyWheelRightID, MotorAlignmentValue.Opposed));
        FlywheelRightMotor.setControl(FlywheelRightMotorRequest);
    }
    public double getRightMotorVelocity(){
        return FlywheelRightMotor.getVelocity().getValueAsDouble();
    }

    public double[] getFlywheelVelocity(){
        double[] FlywheelVelocity = {this.getRightMotorVelocity(),this.getLeftMotorVelocity()};
        
        return FlywheelVelocity;
    }



}
